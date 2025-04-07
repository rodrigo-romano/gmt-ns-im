use gmt_dos_actors::actorscript;
use gmt_dos_clients::{gif, signals::Signals};
use gmt_dos_clients_crseo::{
    OpticalModel,
    crseo::{FromBuilder, Gmt},
    sensors::NoSensor,
};
use gmt_dos_clients_io::optics::{M1Modes, Wavefront};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let on_axis = OpticalModel::<NoSensor>::builder()
        .gmt(Gmt::builder().m1(
            gmt_ns_im::config::m1::segment::MODES,
            gmt_ns_im::config::m1::segment::N_MODE,
        ))
        .build()?;
    let on_axis_wavefront: gif::Frame<f64> = gif::Frame::new("on-axis_wavefront.png", 512);
    let i_mode = 9;
    let m1_bm_cmd: Vec<_> = vec![vec![0.; gmt_ns_im::config::m1::segment::N_MODE]; 7]
        .into_iter()
        .flat_map(|mut c| {
            c[i_mode] = 100e-9;
            c
        })
        .collect();
    let m1_bmc = Signals::from((m1_bm_cmd, 1));
    actorscript!(
        1: m1_bmc[M1Modes] -> on_axis[Wavefront] -> on_axis_wavefront
    );
    Ok(())
}
