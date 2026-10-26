use bevy_ecs::schedule::IntoScheduleConfigs;
use frc::ecs::SimulatorPeriodic;
use frc::math::plant::DCMotor;
use frc::prelude::*;
use frc::sim::prelude::*;
use frc::sim::runtime::SimRuntime;

use mujoco_rs::mujoco_c::mjtTrn;
use mujoco_rs::prelude::*;
use mujoco_rs::viewer::MjViewer;
use mujoco_rs::wrappers::mj_editing::DcMotorConfig;

pub struct MuJoCoRuntime {
    spec: MjSpec,
}

impl MuJoCoRuntime {
    pub fn new(xml: String) -> Self {
        Self {
            spec: MjSpec::from_xml_string(&xml).expect("Unable to load MuJoCo model from XML."),
        }
    }

    pub fn with_motor(
        mut self,
        name: &str,
        joint: &str,
        motor_type: MotorType,
        gearing: f64,
    ) -> Self {
        let plant = DCMotor::from_motor_type(motor_type);
        let actuator = self
            .spec
            .add_actuator()
            .with_name(name)
            .with_target(joint)
            .with_trntype(mjtTrn::mjTRN_JOINT);
        actuator.with_gear([gearing, 0.0, 0.0, 0.0, 0.0, 0.0]);
        actuator.with_ctrlrange([-12.0, 12.0]);
        actuator
            .set_to_dc_motor(
                DcMotorConfig::default()
                    .with_resistance(plant.resistance.value)
                    .with_motorconst([plant.kt.value, 1.0 / plant.kv.value])
                    .with_nominal([
                        plant.nominal_voltage.value,
                        plant.stall_torque.value,
                        plant.free_speed.value,
                    ])
                    .with_saturation([plant.stall_torque.value, 0.0, 0.0]),
            )
            .unwrap();
        self
    }
}

impl SimRuntime for MuJoCoRuntime {
    const NAME: &'static str = "MuJoCo Runtime";

    fn init(&mut self, state: &mut State<Self>) {
        let model = self.spec.compile().expect("Unable to load MuJoCo model.");
        let viewer =
            MjViewer::launch_passive(&model, 0).expect("Could not launch the simulation viewer.");
        let data = MjData::new(Box::new(model));
        state.insert_resource(Data(data));
        state.insert_resource(viewer);

        state.add_systems(SimulatorPeriodic, (motor_update, gyro_update, step).chain());
    }
}

#[derive(Debug, Deref, DerefMut)]
struct Data(MjData<Box<MjModel>>);

fn step(mut data: ResourceMut<Data>, mut viewer: ResourceMut<MjViewer>) {
    if viewer.running() {
        viewer.sync_data(&mut data);
        viewer.render().unwrap();
    }

    let steps = (20.0.milliseconds() / data.model_opt().timestep.seconds()).value as usize;
    for _ in 0..steps {
        data.step();
    }
}

fn motor_update(mut data: ResourceMut<Data>) {
    let inputs = &SimulatorInputs::get().motors;
    let mut outputs = SimulatorOutputs::get();

    for ((bus, id), input) in inputs {
        let output = outputs.motor(bus, *id);

        let name = format!("{bus}:{id}");
        let Some(actuator) = data.actuator(&name) else {
            warn!("Motor not present in model: '{name}'");
            continue;
        };
        let mut actuator = actuator.view_mut(&mut data);
        actuator.ctrl[0] = input.applied_voltage.value;

        output.supply_voltage = 12.0.volts();
        output.position = input.gearing * actuator.length[0].radians();
        output.velocity = input.gearing * actuator.velocity[0].radians_per_second();
    }
}

fn gyro_update(data: ResourceMut<Data>) {
    let inputs = &SimulatorInputs::get().gyros;
    let mut outputs = SimulatorOutputs::get();

    for ((bus, id), _) in inputs {
        let output = outputs.gyro(bus, *id);

        let name = format!("{bus}:{id}");
        let Some(sensor) = data.sensor(&name) else {
            warn!("Gyro not present in model: '{name}'");
            continue;
        };
        let sensor = sensor.view(&data);

        output.supply_voltage = 12.0.volts();
        output.xvel = sensor.data[0].radians_per_second();
        output.yvel = sensor.data[1].radians_per_second();
        output.zvel = sensor.data[2].radians_per_second();
    }
}
