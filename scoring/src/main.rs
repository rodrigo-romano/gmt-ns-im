use std::env;

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{gif, print::Print};
use gmt_dos_clients_crseo::{
    OpticalModel,
    crseo::{FromBuilder, Gmt, builders::AtmosphereBuilder},
    sensors::{Camera, NoSensor},
};
use gmt_dos_clients_io::optics::{
    Frame, Host, M1State, M2State, PSSn, SegmentPiston, SegmentTipTilt, SegmentWfeRms, TipTilt,
    WfeRms,
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_transceiver::{Monitor, Transceiver};
use gmt_dos_systems_agws::Agws;
use interface::{
    optics::{OpticsState, state::OpticalState},
    units::Mas,
};
use scopes::*;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let sim_sampling_frequency = 1000;

    // AGWS
    let agws_wss = {
        let agws = if config::ATMOSPHERE {
            Agws::<{ config::agws::sh48::RATE }, { config::agws::sh24::RATE }>::builder()
                .load_atmosphere("atmosphere/atmosphere.toml", sim_sampling_frequency as f64)?
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
    let on_axis_wavefront: gif::Gif<f64> =
        gif::Gif::new("on-axis_wavefront.gif", 512, 512)?.delay(200);

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
    .sampling_frequency(sim_sampling_frequency as f64)
    .with_pssn()
    .build()?;

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
    let aprint = Print::new(6);
    let optical_state = OpticalState::default();

    actorscript! {
        #[model(name=scoring)]
    1: gmt_state_rx[OpticsState] -> on_axis
    1: on_axis[WfeRms<-9>].. -> shub
    1: on_axis[SegmentWfeRms<-9>].. -> shub
    1: on_axis[SegmentPiston<-9>].. -> shub
    1: on_axis[Mas<TipTilt>].. -> shub
    1: on_axis[Mas<SegmentTipTilt>].. -> shub
    1000: on_axis[Frame<Host>]${512*512}..
    1000: on_axis[PSSn] -> aprint
    // 1000: on_axis[Wavefront].. -> on_axis_wavefront

    1: gmt_state_rx[OpticsState] -> optical_state [M1State] -> m1_lom
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
