// src/http_server.rs

use crate::config::{
    PICOSERVE_HTTP_BUFFER, PICOSERVE_TCP_RX_BUFFER, PICOSERVE_TCP_TX_BUFFER, WEB_PORT,
    WEB_TASK_POOL_SIZE,
};
use defmt::{error, info};
use embassy_executor::task;
use embassy_net::Stack;
use embassy_time::Duration;
use picoserve::{
    AppBuilder, AppRouter,
    response::{WebSocketUpgrade, ws},
    routing::{get, get_service},
};

// ##################################################
// #      DEFINICJA APLIKACJI WEBOWEJ (PICOSERVE)
// ##################################################

pub struct AppProps;

impl AppBuilder for AppProps {
    type PathRouter = impl picoserve::routing::PathRouter;

    fn build_app(self) -> picoserve::Router<Self::PathRouter> {
        picoserve::Router::new()
            .route(
                "/",
                get_service(picoserve::response::File::html(include_str!(
                    "../web/index.html" // Zaktualizowana ścieżka
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
                    upgrade.on_upgrade(WebsocketEcho).with_protocol("echo")
                }),
            )
    }
}

// ##################################################
// #      LOGIKA WEBSOCKET
// ##################################################

struct WebsocketEcho;

impl ws::WebSocketCallback for WebsocketEcho {
    async fn run<R: embedded_io_async::Read, W: embedded_io_async::Write<Error = R::Error>>(
        self,
        mut rx: ws::SocketRx<R>,
        mut tx: ws::SocketTx<W>,
    ) -> Result<(), W::Error> {
        info! {"WsRq"}; // Żądanie WebSocket
        let mut buffer = [0; 1024];

        let close_reason = loop {
            match rx.next_message(&mut buffer).await {
                Ok(ws::Message::Text(data)) => tx.send_text(data).await,
                Ok(ws::Message::Binary(data)) => tx.send_binary(data).await,
                Ok(ws::Message::Close(reason)) => {
                    info!("Websocket close reason: {}", reason);
                    break None;
                }
                Ok(ws::Message::Ping(data)) => tx.send_pong(data).await,
                Ok(ws::Message::Pong(_)) => continue,
                Err(err) => {
                    error!("Websocket Error");

                    let code = match err {
                        ws::ReadMessageError::Io(err) => return Err(err),
                        ws::ReadMessageError::ReadFrameError(_)
                        | ws::ReadMessageError::MessageStartsWithContinuation
                        | ws::ReadMessageError::UnexpectedMessageStart => 1002,
                        ws::ReadMessageError::ReservedOpcode(_) => 1003,
                        ws::ReadMessageError::TextIsNotUtf8 => 1007,
                    };

                    break Some((code, "Websocket Error"));
                }
            }?;
        };

        tx.close(close_reason).await
    }
}

// ##################################################
// #      TASK URUCHAMIAJĄCY SERWER
// ##################################################

/// Task puli serwera Picoserve.
/// Uruchamia `WEB_TASK_POOL_SIZE` instancji tego taska.
#[task(pool_size = WEB_TASK_POOL_SIZE)]
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
