use std::{
    env,
    fs::{self, File},
    path::Path,
};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    gif::{Frame, Gif},
    integrator::Integrator,
    print::Print,
    sampler::Sampler,
    timer::Timer,
};
use gmt_dos_clients_crseo::{
    OpticalModel, OpticalModelBuilder,
    calibration::Reconstructor,
    crseo::{FromBuilder, Gmt},
    sensors::{NoSensor, WaveSensor},
};
use gmt_dos_clients_io::{
    Estimate, gmt_m1,
    gmt_m2::M2RigidBodyMotions,
    optics::{Wavefront, WfeRms},
};
use gmt_dos_clients_optics_state::{
    M1State, M2State, MirrorState, OpticalState, OpticsState, SegmentState,
};
use gmt_dos_systems_agws::{
    agws::sh24::Sh24TT,
    builder::shack_hartmann::{AgwsGuideStar, ShackHartmannBuilder},
    kernels::{Kernel, KernelFrame},
};
use interface::{Data, Read, Tick, UniqueIdentifier, Update, Write, filing::Filing};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(&env::var("DATA_REPO")?).join("qp").join("sh24");
    fs::create_dir_all(&data_repo)?;
    unsafe {
        env::set_var("DATA_REPO", data_repo);
    }

    let recon: Reconstructor = serde_pickle::from_reader(
        File::open("../calibrations/sh24/recon_sh24-to-rbm_pth.pkl")?,
        Default::default(),
    )?;
    println!("{recon}");
    let gmtb = Gmt::builder().m1(
        config::m1::segment::RAW_MODES,
        config::m1::segment::N_RAW_MODE,
    );
    let sh24 = OpticalModelBuilder::from(
        &ShackHartmannBuilder::<Reconstructor>::sh24().use_calibration_src(),
    )
    .gmt(gmtb.clone())
    .build()?;
    let sh24_kern = Kernel::<Sh24TT<1>>::try_from(
        ShackHartmannBuilder::<Reconstructor>::sh24().reconstructor(recon),
    )?
    .controller(Integrator::<M2RigidBodyMotions>::new(42).gain(0.5));
    let sh48_wave = OpticalModel::<WaveSensor>::builder()
        .source(AgwsGuideStar::sh48())
        .sensor(
            OpticalModel::<NoSensor>::builder()
                .source(AgwsGuideStar::sh48())
                .into(),
        )
        .build()?;

    let mirror = MirrorState::default()
        .set_segment_state(1, SegmentState::rbms(&[0., 0., 0., 0., 1e-6, 0.]));
    // .set_segment_state(2, SegmentState::rbms(&[1e-5, 0., 0., 0., 0., 0.]))
    // .set_segment_state(7, SegmentState::rbms(&[1e-5, 0., 0., 0., 0., 0.]));
    let optical_state = OpticalState::default().zero_point(OpticalState::m1(mirror));
    let m2_state = MirrorState::default();

    let print = Print::default().tag("WFE RMS [nm]");

    let timer: Timer = Timer::new(20);

    let on_axis = OpticalModel::<NoSensor>::builder().build()?;

    let sh24_frame = Frame::<f32>::new("sh24-frame.gif", 24 * 12);
    let sampler = Sampler::default();

    let onaxis_wavefront_gif = Gif::new("on-axis_wavefront.gif", 512, 512)?;
    let sh48_wavefront_gif = Gif::new("sh48_wavefront.gif", 512 * 3, 512)?;

    let m1 = MirrorState::default();
    let m1_scopes = M1RBMScope::new()?;

    let m2 = MirrorState::default();
    let m2_scopes = M2RBMScope::new()?;

    type Sh24Frame = KernelFrame<Sh24TT<1>>;
    // type AgwsSh24 = Sh24<1>;
    // type AgwsSh24Kernel = Sh24Kern<Sh24TT<1>>;

    actorscript!(
        #[model(name=agws_sh24)]
        #[labels(
            timer="💓",
            sh24="GMT\nAGWS SH24",
            sh24_kern="AGWS SH24\nKernel",
            on_axis="On-axis\nGMT",
            sh48_wave="SH48 GMT\nWave-Sensor",
            sampler = "1:10",
            sh24_frame = "SH24\nframe")]
        1: timer[Tick] -> optical_state[OpticsState]
            -> sh24[Sh24Frame]! -> sh24_kern[M2RigidBodyMotions] -> m2_state[M2State]
                -> optical_state[OpticsState] -> on_axis[WfeRms<-9>] -> print
        1: sh24[Sh24Frame] -> sampler
        10: sampler[Sh24Frame] -> sh24_frame
        1: on_axis[Wavefront] -> onaxis_wavefront_gif
        1: optical_state[OpticsState] -> sh48_wave[Wavefront] -> sh48_wavefront_gif
        1: optical_state[M1State] -> m1[M1RBM<1>].. -> m1_scopes
        1: optical_state[M2State] -> m2[M2RBM<1>].. -> m2_scopes
    );

    {
        use gmt_dos_clients_crseo::{
            OpticalModelBuilder,
            calibration::{CalibrationMode, ClosedLoopCalibration, ClosedLoopReconstructor},
            centroiding::CentroidsProcessing,
            crseo::{Imaging, gmt::GmtM2},
            sensors::builders::CameraBuilder,
        };
        use gmt_dos_clients_io::Estimate;
        use gmt_dos_systems_agws::agws::sh48::Sh48;

        const R: usize = 10;

        let sh48_omb: OpticalModelBuilder<CameraBuilder<1>> =
            (&ShackHartmannBuilder::<Reconstructor>::sh48().use_calibration_src()).into();
        let sh24_omb: OpticalModelBuilder<CameraBuilder<1>> =
            (&ShackHartmannBuilder::<Reconstructor>::sh24().use_calibration_src()).into();
        let recon: ClosedLoopReconstructor = if let Ok(recon) =
            ClosedLoopReconstructor::from_data_repo("sh48_closed-loop_Txy_calib.bin")
        {
            recon
        } else {
            let mut recon =
                <CentroidsProcessing as ClosedLoopCalibration<GmtM2, Imaging>>::calibrate(
                    &(&sh48_omb).into(),
                    CalibrationMode::t_xy(1e-6),
                    &(&sh24_omb).into(),
                    CalibrationMode::r_xy(1e-6),
                )?;
            recon
                .pseudoinverse()
                .to_data_repo("sh48_closed-loop_Txy_calib.bin")?;
            recon
        };
        println!("{recon}");
        let sh48 = OpticalModelBuilder::from(
            &ShackHartmannBuilder::<ClosedLoopReconstructor, R>::sh48().use_calibration_src(),
        )
        .gmt(gmtb.clone())
        .build()?;
        let sh48_kern = Kernel::<Sh48<R>>::try_from(
            ShackHartmannBuilder::<ClosedLoopReconstructor, R>::sh48().reconstructor(recon),
        )?
        .controller(Integrator::<Estimate>::new(42).gain(0.5));

        let merge_agws = MergeAgws::new();

        let timer: Timer = Timer::new(200);

        type Sh48Frame = KernelFrame<Sh48<R>>;

        actorscript!(
            #[model(name=agws)]
            #[labels(
                timer="💓",
                sh24="GMT\nAGWS SH24",
                sh24_kern="AGWS SH24\nKernel",
                sh48="GMT\nAGWS SH48",
                sh48_kern="AGWS SH48\nKernel",
                on_axis="On-axis\nGMT",
                sh48_wave="SH48 GMT\nWave-Sensor",
                sampler = "1:10",
                sh24_frame = "SH24\nframe")]
            1: timer[Tick] -> optical_state[OpticsState]
                -> sh24[Sh24Frame]! -> sh24_kern[M2RigidBodyMotions] -> merge_agws[M2State]
                    -> optical_state[OpticsState] -> on_axis[WfeRms<-9>] -> print
            1: optical_state[OpticsState] -> sh48
            10: sh48[Sh48Frame]! -> sh48_kern[Estimate]
                    -> merge_agws
            1: sh24[Sh24Frame] -> sampler
            10: sampler[Sh24Frame] -> sh24_frame
            1: on_axis[Wavefront] -> onaxis_wavefront_gif
            1: optical_state[OpticsState] -> sh48_wave[Wavefront] -> sh48_wavefront_gif
            1: optical_state[M1State] -> m1[M1RBM<1>].. -> m1_scopes
            1: optical_state[M2State] -> m2[M2RBM<1>].. -> m2_scopes
        );

        let mut m1_scopes_lock = m1_scopes.lock().await;
        let mut m2_scopes_lock = m2_scopes.lock().await;
        println!("Running... press Ctrl-C to stop");
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("Caught Ctrl-C, shutting down.");
            }
            _ = m1_scopes_lock.close() => {}
            _ = m2_scopes_lock.close() => {}
        }
    }

    Ok(())
}

pub struct MergeAgws {
    m2_rbms: Vec<f64>,
}
impl MergeAgws {
    pub fn new() -> Self {
        Self {
            m2_rbms: vec![0f64; 42],
        }
    }
}
impl Update for MergeAgws {}
impl Read<M2RigidBodyMotions> for MergeAgws {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        // dbg!(&data);
        self.m2_rbms
            .chunks_mut(6)
            .zip(data.chunks(6))
            .for_each(|(rbms, data)| {
                rbms[3] = data[3];
                rbms[4] = data[4];
            });
    }
}
impl Read<Estimate> for MergeAgws {
    fn read(&mut self, data: Data<Estimate>) {
        // dbg!(&data);
        self.m2_rbms
            .chunks_mut(6)
            .zip(data.chunks(6))
            .for_each(|(rbms, data)| {
                rbms[0] = data[0];
                rbms[1] = data[1];
            });
    }
}
impl Write<M2State> for MergeAgws {
    fn write(&mut self) -> Option<Data<M2State>> {
        Some(Data::new(MirrorState::from_rbms(&self.m2_rbms).into()))
    }
}

pub enum M1RBM<const ID: u8> {}
impl<const ID: u8> UniqueIdentifier for M1RBM<ID> {
    type DataType = <gmt_m1::segment::RBM<ID> as UniqueIdentifier>::DataType;
    const PORT: u16 = 51_110 + ID as u16;
}
impl<const ID: u8> Write<M1RBM<ID>> for MirrorState {
    fn write(&mut self) -> Option<Data<M1RBM<ID>>> {
        <_ as Write<gmt_m1::segment::RBM<ID>>>::write(self).map(|data| data.transmute())
    }
}

#[gmt_dos_clients_scope::scopehub]
pub enum M1RBMScope {
    Scope(M1RBM<1>),
    Scope(M1RBM<2>),
    Scope(M1RBM<3>),
    Scope(M1RBM<4>),
    Scope(M1RBM<5>),
    Scope(M1RBM<6>),
    Scope(M1RBM<7>),
}

pub enum M2RBM<const ID: u8> {}
impl<const ID: u8> UniqueIdentifier for M2RBM<ID> {
    type DataType = <gmt_m1::segment::RBM<ID> as UniqueIdentifier>::DataType;
    const PORT: u16 = 52_220 + ID as u16;
}
impl<const ID: u8> Write<M2RBM<ID>> for MirrorState {
    fn write(&mut self) -> Option<Data<M2RBM<ID>>> {
        <_ as Write<gmt_m1::segment::RBM<ID>>>::write(self).map(|data| data.transmute())
    }
}

#[gmt_dos_clients_scope::scopehub]
pub enum M2RBMScope {
    Scope(M2RBM<1>),
    Scope(M2RBM<2>),
    Scope(M2RBM<3>),
    Scope(M2RBM<4>),
    Scope(M2RBM<5>),
    Scope(M2RBM<6>),
    Scope(M2RBM<7>),
}
