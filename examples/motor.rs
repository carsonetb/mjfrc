use frc::prelude::*;
use frc::sim::prelude::*;
use mjfrc::MuJoCoRuntime;

fn spin(mut motor: ResourceMut<TalonFX>) {
    motor.set_voltage(12.0.volts());
    motor.simulate();
    dbg!(motor.get_velocity());
}

fn main() {
    let xml = include_str!("motor.xml");

    let mut state: State<MuJoCoRuntime> = State::new().unwrap();
    state.set_runtime(MuJoCoRuntime::new(xml.into()).with_motor(
        "*:1",
        "hinge",
        MotorType::KrakenX60,
        1.0,
    ));

    state.insert_resource(TalonFX::new(1, "*"));
    state.add_systems(Periodic, spin);

    state.run();
}
