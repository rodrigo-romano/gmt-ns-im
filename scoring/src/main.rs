use std::{env, fs, path::Path};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{gif, print::Print};
use gmt_dos_clients_crseo::{
    OpticalModel,
    crseo::{
        Atmosphere, FromBuilder, Gmt, RayTracing, builders::AtmosphereBuilder, imaging::Detector,
    },
    sensors::Camera,
};
use gmt_dos_clients_io::optics::{
    Dev, Frame, Host, PSSn, SegmentPiston, SegmentTipTilt, SegmentWfeRms, TipTilt, Wavefront,
    WfeRms,
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_optics_state::{M1State, M2State, OpticalState, OpticsState};
use gmt_dos_clients_transceiver::{Monitor, Transceiver};
use gmt_dos_systems_agws::Agws;
use interface::units::Mas;
use scopes::*;
use skyangle::Conversion;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing::subscriber::set_global_default(
        tracing_subscriber::FmtSubscriber::builder()
            .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
            .finish(),
    )?;

    let data_repo = Path::new(&env::var("DATA_REPO")?)
        .join("main")
        .join("scoring");
    fs::create_dir_all(&data_repo)?;
    unsafe {
        env::set_var("DATA_REPO", data_repo);
    }

    let sim_sampling_frequency = 1000;

    // AGWS
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("atmosphere.bin");
    let atm = Atmosphere::builder().ray_tracing(
        RayTracing::default()
            .n_width_px(865)
            .field_size(10f64.from_arcmin())
            .duration(30f64)
            .filepath(path.as_os_str())
            .n_duration(5),
    );
    let agws_wss = {
        let agws = if config::ATMOSPHERE {
            Agws::<{ config::agws::sh48::RATE }, { config::agws::sh24::RATE }>::builder()
                // .load_atmosphere("atmosphere/atmosphere.toml", sim_sampling_frequency as f64)?
                .atmosphere(atm, sim_sampling_frequency as f64)
        } else {
            Agws::<{ config::agws::sh48::RATE }, { config::agws::sh24::RATE }>::builder()
        }
        .gmt(Gmt::builder().m1(
            config::m1::segment::RAW_MODES,
            config::m1::segment::N_RAW_MODE,
        ));
        agws.wave_sensor().build()?
    };
    // if let Some(p24) = config::agws::sh24::POINTING_ERROR {
    //     let _ = agws.sh24_pointing(p24).await;
    // }
    // println!("{agws}");
    // println!("{agws_wss}");

    // let sh48_frame: gif::Frame<f32> = gif::Frame::new("sh48_frame.png", 48 * 8);
    // let sh24_frame: gif::Frame<f32> = gif::Frame::new("sh24_frame.png", 24 * 12);
    // let on_axis_wavefront: gif::Frame<f64> = gif::Frame::new("on-axis_wavefront.png", 512);
    let agws_wavefronts: gif::Frame<f64> = gif::Frame::new("agws_wavefronts.png", 512);
    let on_axis_wavefront: gif::Gif<f64> = gif::Gif::new("on-axis_wavefront.gif", 512, 512)?;
    let on_axis_frame: gif::Gif<f32> = gif::Gif::new("on-axis_frame.gif", 512, 512)?;

    // On-axis scoring star
    let atm = AtmosphereBuilder::load("../atmosphere/atmosphere.toml")?;
    let on_axis = if config::ATMOSPHERE {
        OpticalModel::<Camera>::builder().atmosphere(atm)
    } else {
        OpticalModel::<Camera>::builder()
    }
    .gmt(Gmt::builder().m1(
        config::m1::segment::RAW_MODES,
        config::m1::segment::N_RAW_MODE,
    ))
    .sensor(Camera::builder().detector(Detector::default().n_px_imagelet(512)))
    .sampling_frequency(sim_sampling_frequency as f64)
    .with_pssn()
    .build()?;
    println!("{on_axis}");

    // Linear Optical Models
    let m1_lom = LinearOpticalModel::new()?;
    let m2_lom = LinearOpticalModel::new()?;

    // SCOPES
    let shub = OnAxisScopes::new()?;
    // let mount_scopes = MountScopes::new()?;

    let m1_scopes = M1Scopes::new()?;
    let m2_scopes = M2Scopes::new()?;
    // ---

    let tx_address = "127.0.0.1";
    let rx_address = "127.0.0.1:0";
    let mut gmt_state_mon = Monitor::new();
    // let m1_state_rx =
    //     Transceiver::<M1State>::receiver(tx_address, rx_address)?.run(&mut gmt_state_mon);
    // let m2_state_rx =
    //     Transceiver::<M2State>::receiver(tx_address, rx_address)?.run(&mut gmt_state_mon);
    let gmt_state_rx =
        Transceiver::<OpticsState>::receiver(tx_address, rx_address)?.run(&mut gmt_state_mon);
    let aprint = Print::<Vec<f64>>::new(6);
    let optical_state = OpticalState::default();

    // let state_print = Print::default().scale(1e9_f64);

    actorscript! {
        #[model(name=scoring)]
        #[labels(gmt_state_rx="🎧")]
    1: gmt_state_rx[OpticsState].. -> on_axis
    1: on_axis[WfeRms<-9>].. -> shub
    1: on_axis[SegmentWfeRms<-9>].. -> shub
    1: on_axis[SegmentPiston<-9>].. -> shub
    1: on_axis[Mas<TipTilt>].. -> shub
    1: on_axis[Mas<SegmentTipTilt>].. -> shub
    1000: on_axis[Frame<Host>].. -> on_axis_frame
    1000: on_axis[PSSn] -> aprint
    1000: on_axis[Wavefront].. -> on_axis_wavefront

    1: gmt_state_rx[OpticsState].. -> optical_state [M1State] -> m1_lom
    1: optical_state [M2State] -> m2_lom
    1: m1_lom[M1SegmentPiston].. -> m1_scopes
    1: m2_lom[M2SegmentPiston].. -> m2_scopes
    1: m1_lom[M1SegmentTipTilt].. -> m1_scopes
    1: m2_lom[M2SegmentTipTilt].. -> m2_scopes

    }

    if env::var("FOREGO_SCOPES").is_err() {
        shub.lock().await.close().await?;
        // (&mut *mount_scopes.lock().await).await?;
        m1_scopes.lock().await.close().await?;
        m2_scopes.lock().await.close().await?;
    }
    Ok(())
}
