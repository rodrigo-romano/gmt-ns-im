use clap::{Args, Parser, Subcommand};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    gif::{Frame, FrameBuilder, Gif},
    print::Print,
    sampler::Sampler,
    timer::Timer,
};
use gmt_dos_clients_crseo::{
    OpticalModel,
    crseo::{FromBuilder, Gmt},
    sensors::NoSensor,
};
use gmt_dos_clients_io::{
    gmt_m2::M2RigidBodyMotions,
    optics::{SensorData, Wavefront, WfeRms},
};
use gmt_dos_clients_optics_state::{
    M1State, MirrorState, OpticalState, OpticsState, arrow::OpticalStateArrow,
};
use gmt_dos_systems_agws::{
    Agws,
    agws::sh48::{Sh48, kernel::Sh48Kern},
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::KernelFrame,
    qp::{ActiveOptics, QP},
};
use interface::Tick;
use nanorand::{Rng, WyRand};
use std::path::Path;

const N_MODE: usize = 271;
const M1_BM: usize = 27;
const M1_RBM: usize = 41;
const M2_RBM: usize = 41;

type K48 = ActiveOptics<1, 41, 41, 27, 271>;

#[derive(Parser)]
struct Cli {
    // /// GMT Mirror ID (1 or 2)
    // #[arg(short, long)]
    // mirror: usize,
    // /// GMT segment ID [1,2,...,7]
    // #[arg(short, long)]
    // segment: usize,
    // /// Simulation sample #
    // #[arg(short, long, default_value_t = 20)]
    // n_sample: usize,
    // #[command(flatten)]
    // rbm: Option<Rbm>,
    // #[command(flatten)]
    // mode: Option<Mode>,
    #[command(subcommand)]
    setup: Setup,
    /// Simulation sample #
    #[arg(short, long, default_value_t = 20)]
    n_sample: usize,
}

#[derive(Subcommand, Clone)]
enum Setup {
    Rand {
        #[arg(long)]
        m1_rbms: Option<f64>,
        #[arg(long)]
        m2_rbms: Option<f64>,
        #[arg(long)]
        m1_modes: Option<f64>,
    },
    Specs {
        /// GMT Mirror ID (1 or 2)
        #[arg(short, long)]
        mirror: usize,
        /// GMT segment ID [1,2,...,7]
        #[arg(short, long)]
        segment: usize,
        #[command(flatten)]
        rbm: Option<Rbm>,
        #[command(flatten)]
        mode: Option<Mode>,
    },
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
    //let data_path = Path::new("/home/ubuntu/projects/im-sim-scripts/aco_loop_example/data");
    let data_path = Path::new("/home/rromano/Workspace/misc/aco-rust-sim/aco_loop_example/data");
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

    match cli.setup {
        Setup::Rand {
            m1_rbms,
            m2_rbms,
            m1_modes,
        } => {
            let mut rng = WyRand::new();
            if let Some(m1) = m1_rbms {
                println!("M1 RBMs (micro units)");
                variates(&mut m1_rbm_buf, &mut rng, m1);
            }
            if let Some(m2) = m2_rbms {
                println!("M2 RBMs (micro units)");
                variates(&mut m2_rbm_buf, &mut rng, m2);
            }
            if let Some(m1) = m1_modes {
                println!("M1 modes (micro units)");
                variates(&mut m1_modes_buf, &mut rng, m1);
            }
        }
        Setup::Specs {
            mirror,
            segment,
            rbm,
            mode,
        } => {
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
        }
    }

    // let Cli {
    //     mirror,
    //     segment,
    //     rbm,
    //     mode,
    //     n_sample,
    // } = cli;

    let m1 = MirrorState::new(m1_rbm_buf, m1_modes_buf);
    let optical_state = OpticalState::default().zero_point(OpticalState::new(
        m1,
        MirrorState::from_rbms(&m2_rbm_buf.into_iter().flatten().collect::<Vec<_>>()),
    ));

    let on_axis = OpticalModel::<NoSensor>::builder().gmt(gmtb).build()?;
    let print = Print::default().tag("WFE RMS [nm]");
    let wavefront_gif = Gif::new("qp-wavefront.gif", 512, 512)?;
    let sensor_data_gif = Gif::new("qp-sensor-data.gif", 48 * 6, 48)?.image_size(250)?;

    let frame = Frame::<f32>::new("sh48-frame.png", 48 * 8);
    let sampler = Sampler::default();

    let timer: Timer = Timer::new(cli.n_sample);
    let optical_state_log =
        OpticalStateArrow::<M1State, M2RigidBodyMotions>::builder().build(M1_BM);

    type AgwsSh48 = Sh48<1>;
    type AgwsSh48Kernel = Sh48Kern<K48>;
    type Sh48Frame = KernelFrame<K48>;

    actorscript!(
      #[model(name=acoqp)]
      #[labels(sampler="1:10", on_axis="On-axis GMT",
      frame="SH48 frame")]
      1: timer[Tick] -> optical_state[OpticsState] -> {agws::AgwsSh48}
      1: optical_state[OpticsState] -> optical_state_log
      1: {agws::AgwsSh48Kernel}[OpticsState] -> optical_state
      1: {agws::AgwsSh48Kernel}[SensorData]!.. -> sensor_data_gif
      1: optical_state[OpticsState] -> on_axis[WfeRms<-9>] -> print
      1: on_axis[Wavefront].. -> wavefront_gif
      1: {agws::AgwsSh48}[Sh48Frame] -> sampler
      10: sampler[Sh48Frame].. -> frame
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

fn variates(buffer: &mut Vec<Vec<f64>>, rng: &mut WyRand, bound: f64) {
    buffer.iter_mut().for_each(|seg| {
        seg.iter_mut().for_each(|v| {
            *v = (rng.generate::<f64>() * 2. - 1.) * bound;
        })
    });
    buffer.iter().enumerate().for_each(|(i, seg)| {
        print!(" {}: ", i + 1);
        seg.iter().for_each(|v| print!("{:+10.3e}", v * 1e6));
        println!("");
    });
}
