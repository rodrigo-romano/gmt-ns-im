use gmt_dos_actors::actorscript;
use gmt_dos_clients::{print::Print, timer::Timer};
use gmt_dos_clients_crseo::{
    OpticalModel, OpticalModelBuilder,
    calibration::Reconstructor,
    crseo::{FromBuilder, Gmt},
    sensors::{Camera, NoSensor},
};
use gmt_dos_clients_io::optics::{SensorData, WfeRms};
use gmt_dos_systems_agws::{
    agws::sh48::Sh48,
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::{Kernel, KernelFrame},
};
use interface::{
    Tick,
    optics::{
        OpticsState,
        state::{MirrorState, OpticalState},
    },
};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let omb = OpticalModelBuilder::from(
        ShackHartmannBuilder::<Reconstructor>::sh48().use_calibration_src(),
    )
    .gmt(Gmt::builder().m1(
        config::m1::segment::RAW_MODES,
        config::m1::segment::N_RAW_MODE,
    ));
    let sh48_kern = Kernel::<Sh48<1>>::new(&omb)?;
    let sh48: OpticalModel<Camera> = omb.build()?;
    println!("{sh48}");
    println!("{sh48_kern}");

    let mut m1_rbm_buf = vec![vec![0f64; 6]; 7];
    m1_rbm_buf[0][0] = 1. * 1.1e-6; // M1S1-Tx:
    m1_rbm_buf[1][1] = 1. * 1.2e-6; // M1S2-Ty:
    m1_rbm_buf[2][3] = 1. * 1.4e-6; // M1S3-Rx:
    m1_rbm_buf[3][4] = 1. * 1.5e-6; // M1S4-Ry:
    m1_rbm_buf[4][2] = 1. * 1.6e-6; // M1S5-Tz:
    m1_rbm_buf[5][5] = 1. * 1.3e-6; // M1S5-Rz:
    m1_rbm_buf[6][5] = 1. * 2e-6; // M1S7-Rz:
    let m1_rbm = m1_rbm_buf.into_iter().flatten().collect::<Vec<f64>>();
    let optical_state = OpticalState::new(MirrorState::from_rbms(&m1_rbm), Default::default());

    let on_axis = OpticalModel::<NoSensor>::builder().build()?;
    let print = Print::default();

    let n_sample = 10;
    let timer: Timer = Timer::new(n_sample);

    type Sh48Frame = KernelFrame<Sh48<1>>;
    actorscript!(
      #[labels(sh48="GMT\nSH48x3")]
      1: optical_state[OpticsState] -> on_axis
      1: optical_state[OpticsState] -> sh48
      1: timer[Tick] -> sh48[Sh48Frame] -> sh48_kern[SensorData]${48*48*2*3}
      1: on_axis[WfeRms<-9>] -> print
    );

    let mut log = model_logging_1.lock().await;
    println!("{}", log);
    log.iter::<_, f64>("SensorData")?.for_each(|x| {
        println!("data[{}]: sum={:.6}", x.len(), x.iter().sum::<f64>());
    });

    Ok(())
}
