// use std::env;

use crseo::{FromBuilder, builders::AtmosphereBuilder, imaging::Detector};
use gmt_dos_actors::actorscript;
use gmt_dos_clients::{Tick, gif, timer::Timer};
use gmt_dos_clients_crseo::{OpticalModel, sensors::Camera};
use gmt_dos_clients_io::optics::{Frame, Host};

const N_FRAME: usize = 10_000;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // let n = env::args().nth(1).map(|x| x.parse::<usize>()).unwrap()?;

    let sim_sampling_frequency = 1000;
    let atm = AtmosphereBuilder::load("atmosphere.toml")?;
    let on_axis = OpticalModel::<Camera<N_FRAME>>::builder()
        .atmosphere(atm)
        .sensor(Camera::<N_FRAME>::builder().detector(Detector::default().n_px_imagelet(512)))
        .sampling_frequency(sim_sampling_frequency as f64)
        .with_pssn()
        .build()?;
    println!("{on_axis}");

    let on_axis_frame: gif::Frame<f32> =
        gif::Frame::new(format!("on-axis_frame_{N_FRAME}.png"), 512);

    let timer: Timer = Timer::new(N_FRAME + 1);
    actorscript!(
        1: timer[Tick] -> on_axis
        10000: on_axis[Frame<Host>]! -> on_axis_frame
    );
    Ok(())
}
