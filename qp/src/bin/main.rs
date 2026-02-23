use clap::{Args, Parser};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{gif::Gif, operator::Operator, print::Print, timer::Timer};
use gmt_dos_clients_crseo::{
    OpticalModel, OpticalModelBuilder,
    calibration::Reconstructor,
    crseo::{FromBuilder, Gmt},
    sensors::{Camera, NoSensor},
};
use gmt_dos_clients_io::optics::{SensorData, Wavefront, WfeRms};
use gmt_dos_systems_agws::{
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::{Kernel, KernelFrame},
    qp::{ActiveOptics, QP},
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

#[derive(Parser)]
struct Cli {
    /// GMT Mirror ID (1 or 2)
    #[arg(short, long)]
    mirror: usize,
    /// GMT segment ID [1,2,...,7]
    #[arg(short, long)]
    segment: usize,
    /// Simulation sample #
    #[arg(short, long, default_value_t = 20)]
    n_sample: usize,
    #[command(flatten)]
    rbm: Option<Rbm>,
    #[command(flatten)]
    mode: Option<Mode>,
}

#[derive(Args, Clone, Default)]
// #[group(required = false, multiple = true)]
struct Rbm {
    /// GMT RBM index of [Tx,Ty,Tx,Rx,Ry,Rx]
    #[arg(short, long)]
    rbm: usize,
    /// GMT RBM value
    #[arg(short, long)]
    value: f64,
}
#[derive(Args, Clone, Default)]
struct Mode {
    /// GMT M1 modal coefficient index
    #[arg(short, long)]
    index: usize,
    /// GMT M1 modal coefficient value
    #[arg(short)]
    b: f64,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let cli = Cli::parse();

    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);
    let omb = OpticalModelBuilder::from(
        &ShackHartmannBuilder::<Reconstructor>::sh48().use_calibration_src(),
    )
    .gmt(gmtb.clone());

    // Active optics control algorithm
    let data_path = Path::new("/home/ubuntu/projects/im-sim-scripts/aco_loop_example/data");
    let aco = QP::<M1_RBM, M2_RBM, 27, N_MODE>::new(
        //"../aco_impl_stdalone/SHAcO_qp_rhoP1e-3_kIp5.rs.pkl")
        //"rustCalib_AcO_rhoP1e-12_kIp5.rs.pkl")
        data_path.join("rustCalib_AcO_rhoP1e-12_kIp5.agws.pickle"),
    )?
    .update_calib("sh48_calibration.pkl")?
    .build()?;

    let sh48_kern =
        Kernel::<ActiveOptics<1, M1_RBM, M2_RBM, M1_BM, N_MODE>>::new(&omb)?.estimator(aco);
    let sh48: OpticalModel<Camera> = omb.build()?;
    println!("{sh48}");
    // println!("{sh48_kern}");

    let mut m1_rbm_buf = vec![vec![0f64; 6]; 7];
    let mut m2_rbm_buf = vec![vec![0f64; 6]; 7];
    let mut m1_modes_buf = vec![vec![0f64; M1_BM]; 7];

    let Cli {
        mirror,
        segment,
        rbm,
        mode,
        n_sample,
    } = cli;
    let Rbm { rbm, value } = rbm.unwrap_or_default();
    let Mode { index, b } = mode.unwrap_or_default();

    match mirror {
        1 => {
            m1_rbm_buf[segment - 1][rbm] = value;
            m1_modes_buf[segment - 1][index] = b;
        }
        2 => {
            m2_rbm_buf[segment - 1][rbm] = value;
        }
        i => panic!("found GMT mirror id={i}, expected 1 or 2"),
    }

    let m1 = MirrorState::new(m1_rbm_buf, m1_modes_buf);
    let optical_state = OpticalState::new(
        m1,
        MirrorState::from_rbms(&m2_rbm_buf.into_iter().flatten().collect::<Vec<_>>()),
    );

    let add = Operator::<OpticalState>::new();

    let on_axis = OpticalModel::<NoSensor>::builder().gmt(gmtb).build()?;
    let print = Print::default();
    let gif = Gif::new("qp-wavefront.gif", 512, 512)?;

    // let n_sample = 50;
    let timer: Timer = Timer::new(n_sample);

    type Sh48Frame = KernelFrame<ActiveOptics<1, M1_RBM, M2_RBM, M1_BM, N_MODE>>;
    actorscript!(
      #[labels(sh48="GMT\nSH48x3",add="Add",
          sh48_kern="QP AcO")]
      1: timer[Tick] -> on_axis[WfeRms<-9>] -> print
      1: on_axis[Wavefront] -> gif
      1: optical_state[Left<OpticsState>] -> add
      1: add[OpticsState] -> on_axis
      1: add[OpticsState] -> sh48
      // 1: timer[Tick] -> sh48[Sh48Frame] -> sh48_kern[SensorData]${48*48*2*3}
      1: sh48[Sh48Frame]! -> sh48_kern[Right<OpticsState>]->add// -> e2o//
      1: sh48_kern[SensorData]${48*48*2*3}
      // 1: e2o[Right<OpticsState>] -> add
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
