// src/http_server.rs

use crate::{
    commands::handle_client_command,
    config::{
        PICOSERVE_HTTP_BUFFER, PICOSERVE_TCP_RX_BUFFER, PICOSERVE_TCP_TX_BUFFER, WEB_PORT,
        WEB_TASK_POOL_SIZE,
    },
    state::{FS_CHANNEL, SENSOR_WATCH, TelemetryPacket},
};
use alloc::{
    format,
    string::{String, ToString},
    vec::Vec,
};
use defmt::{error, info, println};
use embassy_executor::task;
use embassy_futures::select::{Either, select};
use embassy_net::Stack;
use embassy_time::Duration;
use embedded_io_async::Read;
use picoserve::{
    AppBuilder, AppRouter,
    extract::FromRequest,
    response::{IntoResponse, StatusCode, WebSocketUpgrade, ws},
    routing::{get, get_service},
};
use serde::{Deserialize, Serialize};
use serde_json::from_slice;

/// Wiadomości wysyłane przez serwer (robota) do klienta.
#[derive(Serialize, Debug)]
enum ServerMessage<'a> {
    /// Potwierdzenie wykonania komendy.
    Ack(bool),
    /// Wiadomość o błędzie (np. zła komenda, błąd implementacji).
    Error { message: &'a str },
    /// Strumień danych telemetrycznych.
    Telemetry(TelemetryPacket),
    // TODO: Dodać odpowiedzi na komendy, np. `FileList(Vec<String>)`
}

#[derive(Deserialize, Debug)]
struct InterfaceData {
    html: String,
}

impl<'r, State> FromRequest<'r, State> for InterfaceData {
    type Rejection = ();

    async fn from_request<R: picoserve::io::Read>(
        _state: &'r State,
        _request_parts: picoserve::request::RequestParts<'r>,
        request_body: picoserve::request::RequestBody<'r, R>,
    ) -> Result<Self, Self::Rejection> {
        let mut rdr = request_body.reader();
        info!("{}", rdr.content_length());
        let mut result: Vec<u8> = Vec::with_capacity(rdr.content_length());
        let mut buf = [0; 100];

        while let Ok(b) = rdr.read(&mut buf).await {
            if b == 0 {
                break;
            }
            info!("Reader read {} bytes", b);
            result.extend_from_slice(&buf[..b]);
        }
        Ok(InterfaceData {
            html: String::from_utf8(result).unwrap(),
        })
    }
}

async fn get_interface() -> impl IntoResponse {
    let (tx, rx) = oneshot::channel();
    let request = crate::state::FilesystemRequest::GetFile("interface.html\0".to_string(), tx);

    FS_CHANNEL.send(request).await;

    match rx.await {
        Ok(buffer) => {
            info!("Serving /interface.html ({} bytes)", buffer.len());
            (
                StatusCode::OK,
                ("Content-Type", "text/html; charset=utf-8"),
                String::from_utf8(buffer).unwrap(),
            )
        }
        Err(_) => {
            error!("Failed to read /interface.html from storage task");
            (
                StatusCode::SEE_OTHER,
                ("Location", "/update"),
                "".to_string(),
            )
        }
    }
}

async fn load_interface(form: InterfaceData) -> impl IntoResponse {
    info!("Trying to save new interface");

    let request = crate::state::FilesystemRequest::WriteFile(
        "interface.html\0".to_string(),
        form.html.clone().into_bytes(),
    );

    FS_CHANNEL.send(request).await;

    info!("Serving new interface as is");
    (StatusCode::OK, ("", ""), form.html.clone())
}
pub struct AppProps;

impl AppBuilder for AppProps {
    type PathRouter = impl picoserve::routing::PathRouter;

    fn build_app(self) -> picoserve::Router<Self::PathRouter> {
        picoserve::Router::new()
            .route("/dynamic", get(get_interface))
            .route(
                "/update",
                get_service(picoserve::response::File::html(include_str!(
                    "../web/update.html" // Zaktualizowana ścieżka
                )))
                .post(load_interface),
            )
            .route(
                "/old",
                get_service(picoserve::response::File::html(include_str!(
                    "../web/index.html" // Zaktualizowana ścieżka
                ))),
            )
            .route(
                "/",
                get_service(picoserve::response::File::html(include_str!(
                    "../web/index2.html" // Zaktualizowana ścieżka
                ))),
            )
            .route(
                "/index.css",
                get_service(picoserve::response::File::css(include_str!(
                    "../web/index.css" // Zaktualizowana ścieżka
                ))),
            )
            .route(
                "/index.js",
                get_service(picoserve::response::File::javascript(include_str!(
                    "../web/index.js" // Zaktualizowana ścieżka
                ))),
            )
            .route(
                "/ws",
                get(|upgrade: WebSocketUpgrade| {
                    upgrade
                        .on_upgrade(WebsocketHandler)
                        .with_protocol("lf-comm")
                }),
            )
    }
}

struct WebsocketHandler;

impl ws::WebSocketCallback for WebsocketHandler {
    async fn run<R: embedded_io_async::Read, W: embedded_io_async::Write<Error = R::Error>>(
        self,
        mut rx: ws::SocketRx<R>,
        mut tx: ws::SocketTx<W>,
    ) -> Result<(), W::Error> {
        info! {"WebSocket connection opened"};
        let mut rx_buffer = [0; 1024];
        let mut _tx_buffer = [0; 256];

        let mut sensor_receiver = SENSOR_WATCH.receiver().unwrap();
        sensor_receiver.changed().await;

        loop {
            let event = select(rx.next_message(&mut rx_buffer), sensor_receiver.changed()).await;

            match event {
                Either::First(Ok(ws::Message::Text(data))) => {
                    let response =
                        match from_slice::<crate::commands::ClientCommand>(data.as_bytes()) {
                            Ok(cmd) => {
                                info!("Parsed command: {=str}", format!("{:?}", cmd));
                                handle_client_command(cmd).await;
                                ServerMessage::Ack(true)
                            }
                            Err(_) => {
                                error!("Failed to deserialize ClientCommand");
                                ServerMessage::Error {
                                    message: "Invalid command format",
                                }
                            }
                        };

                    match serde_json::to_vec(&response) {
                        Ok(resp) => {
                            if tx
                                .send_text(core::str::from_utf8(&resp).unwrap())
                                .await
                                .is_err()
                            {
                                break;
                            }
                        }
                        Err(_) => error!("Failed to serialize response"),
                    }
                }

                Either::Second(_) => {
                    let sensor_data = sensor_receiver.get().await;
                    let telemetry_msg = ServerMessage::Telemetry(TelemetryPacket {
                        sensor_values: sensor_data, //normalized_sensors: &sensor_data,
                    });

                    match serde_json::to_vec(&telemetry_msg) {
                        Ok(resp) => {
                            if tx
                                .send_text(core::str::from_utf8(&resp).unwrap())
                                .await
                                .is_err()
                            {
                                break;
                            }
                        }
                        Err(_) => error!("Failed to serialize telemetry"),
                    }
                }

                Either::First(Ok(ws::Message::Close(reason))) => {
                    info!("WebSocket close requested: {}", reason);
                    break;
                }
                Either::First(Err(e)) => {
                    println!("{=str}", format!("{:?}", e));
                    error!("WebSocket Read Error");
                    break;
                }
                Either::First(Ok(_)) => {
                    // Ignored
                }
            }
        }

        info!("WebSocket connection closing.");
        tx.close(None).await?;
        Ok(())
    }
}

#[task(pool_size = WEB_TASK_POOL_SIZE * 2)]
pub async fn web_task(
    id: usize,
    stack: Stack<'static>,
    app: &'static AppRouter<AppProps>,
    config: &'static picoserve::Config<Duration>,
) -> ! {
    let port = WEB_PORT;
    let mut tcp_rx_buffer = [0; PICOSERVE_TCP_RX_BUFFER];
    let mut tcp_tx_buffer = [0; PICOSERVE_TCP_TX_BUFFER];
    let mut http_buffer = [0; PICOSERVE_HTTP_BUFFER];

    picoserve::listen_and_serve(
        id,
        app,
        config,
        stack,
        port,
        &mut tcp_rx_buffer,
        &mut tcp_tx_buffer,
        &mut http_buffer,
    )
    .await
}
