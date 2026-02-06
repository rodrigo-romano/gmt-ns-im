use gmt_dos_actors::actorscript;
use gmt_dos_clients::{gif::Gif, operator::Operator, print::Print, timer::Timer};
use gmt_dos_clients_crseo::{
    OpticalModel, OpticalModelBuilder,
    calibration::Reconstructor,
    crseo::{FromBuilder, Gmt},
    sensors::{Camera, NoSensor},
};
use gmt_dos_clients_io::{
    Estimate,
    optics::{SensorData, Wavefront, WfeRms},
};
use gmt_dos_systems_agws::{
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::{Kernel, KernelFrame},
    qp::{AcO, Estimate2OpticsState, QP},
};
use interface::{
    Left, Right, Tick,
    optics::{
        OpticsState,
        state::{MirrorState, OpticalState},
    },
};
use std::path::Path;

const N_MODE: usize = 271;
const M1_BM: usize = 27;
const M1_RBM: usize = 41;
const M2_RBM: usize = 41;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);
    let omb = OpticalModelBuilder::from(
        ShackHartmannBuilder::<Reconstructor>::sh48().use_calibration_src(),
    )
    .gmt(gmtb.clone());

    // Active optics control algorithm
    let data_path = Path::new("/home/ubuntu/projects/im-sim-scripts/aco_loop_example/data");
    let aco = QP::<M1_RBM, M2_RBM, 27, N_MODE>::new(
        //"../aco_impl_stdalone/SHAcO_qp_rhoP1e-3_kIp5.rs.pkl")
        //"rustCalib_AcO_rhoP1e-12_kIp5.rs.pkl")
        data_path.join("rustCalib_AcO_rhoP1e-12_kIp5.agws.pickle"),
    )
    .unwrap()
    .update_calib("sh48_calibration.pkl")
    .unwrap()
    .build();

    let sh48_kern = Kernel::<AcO<1, M1_RBM, M2_RBM, M1_BM, N_MODE>>::new(&omb)?.estimator(aco);
    let sh48: OpticalModel<Camera> = omb.build()?;
    println!("{sh48}");
    // println!("{sh48_kern}");

    let mut m1_rbm_buf = vec![vec![0f64; 6]; 7];
    m1_rbm_buf[0][0] = 1. * 1.1e-6; // M1S1-Tx:
    m1_rbm_buf[1][1] = 1. * 1.2e-6; // M1S2-Ty:
    m1_rbm_buf[2][3] = 1. * 1.4e-6; // M1S3-Rx:
    m1_rbm_buf[3][4] = 1. * 1.5e-6; // M1S4-Ry:
    m1_rbm_buf[4][2] = 1. * 1.6e-6; // M1S5-Tz:
    m1_rbm_buf[5][5] = 1. * 1.3e-6; // M1S5-Rz:
    m1_rbm_buf[6][5] = 1. * 2e-6; // M1S7-Rz:
    // let m1_rbm = m1_rbm_buf.into_iter().flatten().collect::<Vec<f64>>();
    let mut m2_rbm_buf = vec![0f64; 42];
    m2_rbm_buf[M2_RBM] = 0. * 3e-4; // M2S7-Rz
    // let m2_rbm = m2_rbm_buf;
    let mut m1_modes_buf = vec![vec![0f64; M1_BM]; 7];
    m1_modes_buf[0][0] = 4e-6;
    m1_modes_buf[0][2] = 5e-6;
    // let m1_modes = m1_modes_buf.into_iter().flatten().collect::<Vec<f64>>();
    let m1 = MirrorState::new(m1_rbm_buf, m1_modes_buf);
    let optical_state = OpticalState::new(m1, MirrorState::from_rbms(&m2_rbm_buf));

    let e2o = Estimate2OpticsState::new();
    let add = Operator::<OpticalState>::new();

    let on_axis = OpticalModel::<NoSensor>::builder().gmt(gmtb).build()?;
    let print = Print::default();
    let gif = Gif::new("qp-wavefront.gif", 512, 512)?;

    let n_sample = 50;
    let timer: Timer = Timer::new(n_sample);

    type Sh48Frame = KernelFrame<AcO<1, M1_RBM, M2_RBM, M1_BM, N_MODE>>;
    actorscript!(
      #[labels(sh48="GMT\nSH48x3",add="Add",e2o="=> Optical State",sh48_kern="QP AcO")]
      1: timer[Tick] -> on_axis[WfeRms<-9>] -> print
      1: on_axis[Wavefront] -> gif
      1: optical_state[Left<OpticsState>] -> add
      1: add[OpticsState] -> on_axis
      1: add[OpticsState] -> sh48
      // 1: timer[Tick] -> sh48[Sh48Frame] -> sh48_kern[SensorData]${48*48*2*3}
      1: sh48[Sh48Frame]! -> sh48_kern[Estimate]${84+27*7} -> e2o//
      1: sh48_kern[SensorData]${48*48*2*3}
      1: e2o[Right<OpticsState>] -> add
    );

    // let mut log = model_logging_1.lock().await;
    // println!("{}", log);
    // log.iter::<_, f64>("Estimate")?.for_each(|x| {
    //     &x[..42].chunks(6).enumerate().for_each(|(_sid, rbm)| {
    //         //println!("M1S{}:", sid + 1);
    //         let (t_xyz, r_xyz) = rbm.split_at(3);
    //         //t_xyz.iter().for_each(|x| print!("{},", x));
    //         //r_xyz.iter().for_each(|x| print!("{},", x));

    //         let tn = t_xyz.iter().map(|x| x * x).sum::<f64>().sqrt();
    //         let rn = r_xyz.iter().map(|x| x * x).sum::<f64>().sqrt();
    //         println!(" {:6.3}, {:6.3}\t", 1e6 * tn, 1e6 * rn);
    //     });

    // println!("data[{}]: sum={:.6}", x.len(), x.iter().sum::<f64>());
    // });

    Ok(())
}
