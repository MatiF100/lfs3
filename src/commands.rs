use serde::{Deserialize, Serialize};

use crate::{control::Actions, state::CONTROL_WATCH};

#[derive(Serialize, Deserialize, Debug)]
pub enum ClientCommand {
    SetLed {
        enabled: bool,
    },
    SetMotor {
        base_speed: Option<u8>,
        turn_speed: Option<u8>,
        max_speed: Option<u8>,
        enabled: Option<bool>,
    },
    SetPid {
        kp: Option<f32>,
        ki: Option<f32>,
        kd: Option<f32>,
    },
    ListFiles,
}

pub async fn handle_client_command(cmd: ClientCommand) {
    match cmd {
        ClientCommand::SetMotor {
            base_speed,
            turn_speed,
            max_speed,
            enabled,
        } => {
            let action = Actions {
                set_speed_base: base_speed,
                set_speed_turn: turn_speed,
                set_speed_max: max_speed,
                enable_motors: enabled,
                ..Default::default()
            };

            CONTROL_WATCH.sender().send(action);
        }
        ClientCommand::SetLed { enabled } => todo!(),
        ClientCommand::SetPid { kp, ki, kd } => {
            let action = Actions {
                set_kp: kp,
                set_kd: kd,
                set_ki: ki,
                calibrate: true,
                ..Default::default()
            };

            CONTROL_WATCH.sender().send(action);
        }
        ClientCommand::ListFiles => todo!(),
    }
}
