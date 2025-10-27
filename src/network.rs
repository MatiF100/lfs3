// src/network.rs

use crate::config::*;
use core::net::Ipv4Addr;
use defmt::{info, warn};
use embassy_executor::{Spawner, task};
use embassy_futures::select::{Either, select};
use embassy_net::tcp::TcpSocket;
use embassy_net::{
    Config, IpListenEndpoint, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4,
};
use embassy_time::{Duration, Timer};
use esp_hal::peripherals;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::Controller;
use esp_radio::wifi::{
    AccessPointConfig, ClientConfig, ModeConfig, WifiApState, WifiController, WifiDevice, WifiEvent,
};

/// Inicjalizuje kontroler Wi-Fi, stosy sieciowe AP i STA oraz uruchamia
/// odpowiednie taski do zarządzania siecią.
///
/// Zwraca stosy sieciowe (AP i STA), które mogą być następnie użyte
/// przez serwery WWW.
pub async fn init_network(
    spawner: Spawner,
    timer_group: TimerGroup<'static, peripherals::TIMG0<'static>>,
    rng: esp_hal::rng::Rng,
    wifi_peripheral: peripherals::WIFI<'static>,
) -> (Stack<'static>, Stack<'static>) {
    // Te peryferia są potrzebne do inicjalizacji esp_radio,
    // nawet jeśli nie są bezpośrednio używane (są 'consumed').
    let _ = timer_group;
    let _ = rng;

    let wifi_init = &*crate::mk_static!(
        Controller<'static>,
        esp_radio::init().expect("Failed to initialize WIFI/BLE controller")
    );

    let (mut wifi_controller, interfaces) = esp_radio::wifi::new(
        &wifi_init,
        wifi_peripheral,
        esp_radio::wifi::Config::default(),
    )
    .expect("Failed to initialize WIFI controller");

    let wifi_sta_device = interfaces.sta;
    let wifi_ap_device = interfaces.ap;

    let ap_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(AP_IP_ADDR, AP_IP_CIDR),
        gateway: Some(AP_IP_ADDR),
        dns_servers: Default::default(),
    });
    let sta_config = embassy_net::Config::dhcpv4(Default::default());

    let (ap_stack, ap_runner) = embassy_net::new(
        wifi_ap_device,
        ap_config,
        crate::mk_static!(StackResources<8>, StackResources::<8>::new()),
        NET_SEED,
    );
    let (sta_stack, sta_runner) = embassy_net::new(
        wifi_sta_device,
        sta_config,
        crate::mk_static!(StackResources<8>, StackResources::<8>::new()),
        NET_SEED,
    );

    let client_config = ModeConfig::ApSta(
        ClientConfig::default()
            .with_ssid(SSID.into())
            .with_password(PASSWORD.into()),
        AccessPointConfig::default().with_ssid(AP_SSID.into()),
    );
    wifi_controller.set_config(&client_config).unwrap();

    // Spawnowanie tasków sieciowych
    spawner.spawn(connection(wifi_controller)).ok();
    spawner.spawn(net_task(ap_runner)).ok();
    spawner.spawn(net_task(sta_runner)).ok();

    // Oczekiwanie na adres IP w trybie STA
    info!("Waiting for STA IP address...");
    let sta_address = loop {
        if let Some(config) = sta_stack.config_v4() {
            let address = config.address.address();
            info!("Got STA IP: {}", address);
            break address;
        }
        Timer::after(Duration::from_millis(500)).await;
    };

    // Oczekiwanie na podniesienie linku AP
    info!("Waiting for AP link up...");
    loop {
        if ap_stack.is_link_up() {
            info!("AP link is up");
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!(
        "Connect to AP `{}` (http://{}:{}/)",
        AP_SSID, AP_IP_ADDR, LEGACY_WEB_PORT
    );
    info!(
        "Or connect to STA network `{}` (http://{}:{}/)",
        SSID, sta_address, LEGACY_WEB_PORT
    );

    (ap_stack, sta_stack)
}

/// Task zarządzający połączeniem Wi-Fi (start, connect, reconnect).
#[task]
async fn connection(mut controller: WifiController<'static>) {
    info!("Start connection task");
    info!("Device capabilities: {:?}", controller.capabilities());

    info!("Starting wifi");
    controller.start_async().await.unwrap();
    info!("Wifi started!");

    loop {
        match esp_radio::wifi::ap_state() {
            WifiApState::Started => {
                info!("About to connect...");
                match controller.connect_async().await {
                    Ok(_) => {
                        info!("STA Connected");
                        // Czekaj na rozłączenie
                        controller.wait_for_event(WifiEvent::StaDisconnected).await;
                        info!("STA disconnected");
                    }
                    Err(e) => {
                        info!("Failed to connect to wifi: {}", e);
                        Timer::after(Duration::from_millis(5000)).await
                    }
                }
            }
            _ => return, // Stan AP inny niż 'Started'
        }
    }
}

/// Task puli dla runnerów embassy-net (jeden dla AP, jeden dla STA).
#[task(pool_size = 2)]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

/// Task uruchamiający stary serwer WWW (ten z `main.rs`).
/// Zachowany dla kompatybilności, Picoserve jest w `http_server.rs`.
#[task]
#[allow(clippy::too_many_lines)] // Ten task jest duży i jest przykładem
pub async fn run_webserver(
    mut ap_stack: embassy_net::Stack<'static>,
    mut sta_stack: embassy_net::Stack<'static>,
) {
    let mut ap_server_rx_buffer = [0; TCP_BUFFER_SIZE];
    let mut ap_server_tx_buffer = [0; TCP_BUFFER_SIZE];
    let mut sta_server_rx_buffer = [0; TCP_BUFFER_SIZE];
    let mut sta_server_tx_buffer = [0; TCP_BUFFER_SIZE];
    let mut sta_client_rx_buffer = [0; TCP_BUFFER_SIZE];
    let mut sta_client_tx_buffer = [0; TCP_BUFFER_SIZE];

    let mut ap_server_socket =
        TcpSocket::new(ap_stack, &mut ap_server_rx_buffer, &mut ap_server_tx_buffer);
    ap_server_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    let mut sta_server_socket = TcpSocket::new(
        sta_stack,
        &mut sta_server_rx_buffer,
        &mut sta_server_tx_buffer,
    );
    sta_server_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    let mut sta_client_socket = TcpSocket::new(
        sta_stack,
        &mut sta_client_rx_buffer,
        &mut sta_client_tx_buffer,
    );
    sta_client_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

    loop {
        info!("(Legacy) Wait for connection...");
        let either_socket = select(
            ap_server_socket.accept(IpListenEndpoint {
                addr: None,
                port: LEGACY_WEB_PORT,
            }),
            sta_server_socket.accept(IpListenEndpoint {
                addr: None,
                port: LEGACY_WEB_PORT,
            }),
        )
        .await;
        let (r, server_socket) = match either_socket {
            Either::First(r) => (r, &mut ap_server_socket),
            Either::Second(r) => (r, &mut sta_server_socket),
        };
        info!("(Legacy) Connected...");

        if let Err(e) = r {
            warn!("(Legacy) connect error: {:?}", e);
            continue;
        }

        use embedded_io_async::Write;
        let mut buffer = [0u8; 1024];
        let mut pos = 0;
        loop {
            match server_socket.read(&mut buffer).await {
                Ok(0) => {
                    info!("(Legacy) AP read EOF");
                    break;
                }
                Ok(len) => {
                    let to_print =
                        unsafe { core::str::from_utf8_unchecked(&buffer[..(pos + len)]) };
                    if to_print.contains("\r\n\r\n") {
                        info!("{}", to_print);
                        break;
                    }
                    pos += len;
                }
                Err(e) => {
                    warn!("(Legacy) AP read error: {:?}", e);
                    break;
                }
            };
        }
        if sta_stack.is_link_up() {
            let remote_endpoint = (Ipv4Addr::new(142, 250, 185, 115), 80); // www.mobile-j.de
            info!("(Legacy) connecting to remote...");
            let r = sta_client_socket.connect(remote_endpoint).await;
            if let Err(e) = r {
                warn!("(Legacy) STA connect error: {:?}", e);
                continue;
            }

            use embedded_io_async::Write;
            let r = sta_client_socket
                .write_all(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
                .await;

            if let Err(e) = r {
                warn!("(Legacy) STA write error: {:?}", e);
                // (Oryginalna logika odpowiedzi błędu pominięta dla zwięzłości)
            } else {
                let r = sta_client_socket.flush().await;
                if let Err(e) = r {
                    warn!("(Legacy) STA flush error: {:?}", e);
                } else {
                    info!("(Legacy) connected!");
                    let mut buf = [0; 1024];
                    loop {
                        match sta_client_socket.read(&mut buf).await {
                            Ok(0) => {
                                info!("(Legacy) STA read EOF");
                                break;
                            }
                            Ok(n) => {
                                if server_socket.write_all(&buf[..n]).await.is_err() {
                                    break;
                                }
                            }
                            Err(e) => {
                                warn!("(Legacy) STA read error: {:?}", e);
                                break;
                            }
                        }
                    }
                }
            }
            sta_client_socket.close();
        } else {
            // Odpowiedź, gdy STA nie jest połączone
            let _ = server_socket
                .write_all(
                    b"HTTP/1.0 200 OK\r\n\r\n\
                    <html><body>\
                    <h1>Hello Rust! STA is not connected.</h1>\
                    </body></html>\r\n",
                )
                .await;
        }
        let _ = server_socket.flush().await;
        Timer::after(Duration::from_millis(1000)).await;
        server_socket.close();
        Timer::after(Duration::from_millis(1000)).await;
        server_socket.abort();
    }
}
