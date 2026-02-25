use clap::{Args, Parser};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    gif::{Frame, Gif},
    print::Print,
    sampler::Sampler,
    timer::Timer,
};
use gmt_dos_clients_crseo::{
    OpticalModel,
    crseo::{FromBuilder, Gmt},
    sensors::NoSensor,
};
use gmt_dos_clients_io::optics::{Wavefront, WfeRms};
use gmt_dos_clients_optics_state::{MirrorState, OpticalState, OpticsState};
use gmt_dos_systems_agws::{
    Agws,
    agws::sh48::{Sh48, kernel::Sh48Kern},
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::KernelFrame,
    qp::{ActiveOptics, QP},
};
use interface::Tick;
use std::path::Path;

const N_MODE: usize = 271;
const M1_BM: usize = 27;
const M1_RBM: usize = 41;
const M2_RBM: usize = 41;

type K48 = ActiveOptics<1, 41, 41, 27, 271>;

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

    // Active optics control algorithm
    let data_path = Path::new("/home/ubuntu/projects/im-sim-scripts/aco_loop_example/data");
    let aco = QP::<M1_RBM, M2_RBM, 27, N_MODE>::new(
        //"../aco_impl_stdalone/SHAcO_qp_rhoP1e-3_kIp5.rs.pkl")
        //"rustCalib_AcO_rhoP1e-12_kIp5.rs.pkl")
        data_path.join("rustCalib_AcO_rhoP1e-12_kIp5.agws.pickle"),
    )?
    .update_calib("sh48_calibration.pkl")?
    .build()?;

    let agws = Agws::<1, 1, K48>::builder()
        .gmt(Gmt::builder().m1(
            config::m1::segment::RAW_MODES,
            config::m1::segment::N_RAW_MODE,
        ))
        .sh48(ShackHartmannBuilder::sh48().use_calibration_src())
        .sh48_calibration(aco)
        .build()?;

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
    let optical_state = OpticalState::default().zero_point(OpticalState::new(
        m1,
        MirrorState::from_rbms(&m2_rbm_buf.into_iter().flatten().collect::<Vec<_>>()),
    ));

    let on_axis = OpticalModel::<NoSensor>::builder().gmt(gmtb).build()?;
    let print = Print::default().tag("WFE RMS [nm]");
    let gif = Gif::new("qp-wavefront.gif", 512, 512)?;

    let frame = Frame::<f32>::new("sh48-frame.png", 48 * 8);
    let sampler = Sampler::default();

    let timer: Timer = Timer::new(n_sample);

    type AgwsSh48 = Sh48<1>;
    type AgwsSh48Kernel = Sh48Kern<K48>;
    type Sh48Frame = KernelFrame<K48>;

    actorscript!(
      #[model(name=acoqp)]
      1: timer[Tick] -> optical_state[OpticsState] -> {agws::AgwsSh48}
      1: {agws::AgwsSh48Kernel}[OpticsState] -> optical_state
      1: optical_state[OpticsState] -> on_axis[WfeRms<-9>] -> print
      1: on_axis[Wavefront] -> gif
      1: {agws::AgwsSh48}[Sh48Frame] -> sampler
      10: sampler[Sh48Frame] -> frame
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
