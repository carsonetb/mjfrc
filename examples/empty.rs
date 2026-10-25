use frc::prelude::*;
use mjfrc::MuJoCoRuntime;

fn main() {
    let xml = include_str!("empty.xml");

    let mut state: State<MuJoCoRuntime> = State::new().unwrap();
    state.set_runtime(MuJoCoRuntime::new(xml.into()));

    state.run();
}
