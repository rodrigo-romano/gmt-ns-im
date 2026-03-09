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
    calibration::{Calib, Calibration, MixedMirrorMode, Reconstructor, algebra::CalibProps},
    crseo::{FromBuilder, Gmt, gmt::GmtM1},
    sensors::{NoSensor, WaveSensor},
};
use gmt_dos_clients_io::{
    gmt_m1::segment::ModeShapes,
    gmt_m2::M2RigidBodyMotions,
    optics::{SensorData, Wavefront, WfeRms},
};
use gmt_dos_clients_optics_state::{
    M1State, M2State, MirrorState, OpticalState, OpticsState, SegmentState,
    arrow::OpticalStateArrow,
};
use gmt_dos_systems_agws::{
    agws::sh24::Sh24TT,
    builder::shack_hartmann::{AgwsGuideStar, ShackHartmannBuilder},
    kernels::{Kernel, KernelFrame},
};
use interface::{Tick, filing::Filing};

use qp::sh24::*;

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
    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, M1_N_MODE);
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
        .gmt(gmtb.clone())
        .source(AgwsGuideStar::sh48())
        .sensor(
            OpticalModel::<NoSensor>::builder()
                .source(AgwsGuideStar::sh48())
                .into(),
        )
        .build()?;

    const SID: u8 = 1;
    let mirror = MirrorState::default()
        .set_segment_state(SID, SegmentState::rbms(&[0., 0., 0., 1e-6, 0., 0.]));
    // .set_segment_state(2, SegmentState::rbms(&[1e-5, 0., 0., 0., 0., 0.]))
    // .set_segment_state(7, SegmentState::rbms(&[1e-5, 0., 0., 0., 0., 0.]));
    let optical_state = OpticalState::m1(MirrorState::default().zeros_modes(M1_N_MODE))
        .zero_point(OpticalState::m1(mirror));
    // let optical_state = OpticalState::default().zero_point(OpticalState::m1(
    //     MirrorState::default().set_segment_state(
    //         1,
    //         SegmentState::modes(vec![0f64; M1_N_MODE]).set_mode(0, 1e-6),
    //     ),
    // ));
    // let optical_state = OpticalState::default().zero_point(OpticalState::new(
    //     MirrorState::default().set_segment_state(
    //         1,
    //         SegmentState::modes(vec![0f64; M1_N_MODE]).set_mode(1, 1e-6),
    //     ),
    //     mirror,
    // ));
    let optical_state_arrow =
        OpticalStateArrow::<M1State, M2RigidBodyMotions>::builder().build(M1_N_MODE);
    let m2_state = MirrorState::default();

    let print = Print::default().tag("WFE RMS [nm]");

    let timer: Timer = Timer::new(20);

    let on_axis = OpticalModel::<NoSensor>::builder()
        .gmt(gmtb.clone())
        .build()?;

    let sh24_frame = Frame::<f32>::new("sh24-frame.png", 24 * 12);
    let sh48_frame = Frame::<f32>::new("sh48-frame.png", 48 * 8);
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
            timer="⏲",
            sh24="GMT\nAGWS SH24",
            sh24_kern="AGWS SH24\nKernel",
            on_axis="On-axis\nGMT",
            sh48_wave="SH48 GMT\nWave-Sensor",
            sampler = "1:10",
            sh24_frame = "SH24\nframe")]
        1: timer[Tick] -> optical_state[OpticsState]
            -> sh24[Sh24Frame]! -> sh24_kern[M2RigidBodyMotions] -> m2_state[M2State]
                -> optical_state[OpticsState] -> on_axis[WfeRms<-9>] -> print
        1: sh24_kern[SensorData]${24*24*2}
        1: sh24[Sh24Frame] -> sampler
        10: sampler[Sh24Frame] -> sh24_frame
        1: on_axis[Wavefront] -> onaxis_wavefront_gif
        1: optical_state[OpticsState] -> sh48_wave[Wavefront] -> sh48_wavefront_gif
        1: optical_state[M1State] -> m1[M1RBM<SID>].. -> m1_scopes
        1: m1[ModeShapes<SID>].. -> m1_scopes
        1: optical_state[M2State] -> m2[M2RBM<SID>].. -> m2_scopes
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

        // closed-loop calibration of M2 Sx Txy with SH48
        let file_name = "sh48_closed-loop_Txy_calib.pkl";
        let recon: ClosedLoopReconstructor =
            if let Ok(recon) = ClosedLoopReconstructor::from_data_repo(file_name) {
                recon
            } else {
                let sh48_omb: OpticalModelBuilder<CameraBuilder<1>> =
                    (&ShackHartmannBuilder::<Reconstructor>::sh48().use_calibration_src()).into();
                let sh24_omb: OpticalModelBuilder<CameraBuilder<1>> =
                    (&ShackHartmannBuilder::<Reconstructor>::sh24().use_calibration_src()).into();
                let mut recon =
                    <CentroidsProcessing as ClosedLoopCalibration<GmtM2, Imaging>>::calibrate(
                        &(&sh48_omb).into(),
                        CalibrationMode::t_xy(1e-6),
                        // CalibrationMode::RBM([Some(1e-6), Some(1e-6), Some(1e-6), None, None, None]),
                        &(&sh24_omb).into(),
                        CalibrationMode::r_xy(1e-6),
                    )?;
                recon.pseudoinverse().to_data_repo(file_name)?;
                recon
            };
        println!("{recon}");

        // calibration of M1 Sx bending modes with SH48
        let file_name = "sh48_bending-modes_calib.pkl";
        let m1_bm_recon: Reconstructor = if let Ok(recon) = Reconstructor::from_data_repo(file_name)
        {
            recon
        } else {
            let sh48_omb: OpticalModelBuilder<CameraBuilder<1>> =
                (&ShackHartmannBuilder::<Reconstructor>::sh48().use_calibration_src()).into();
            let mut recon = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
                &(&sh48_omb.gmt(gmtb.clone())).into(),
                CalibrationMode::modes(M1_N_MODE, 1e-6),
            )?;
            recon.pseudoinverse().to_data_repo(file_name)?;
            recon
        };
        println!("{m1_bm_recon}");

        // recon.merge(m1_bm_recon).pseudoinverse();
        // println!("{recon}");
        // let mut c_txy: Vec<_> = recon
        //     .calib()
        //     .map(|c| c.m1_closed_loop_to_sensor().clone())
        //     .collect();
        // let c_bms = m1_bm_recon.calib_slice().to_vec();
        let mmode = MixedMirrorMode::from(vec![
            CalibrationMode::t_xy(1e-6),
            CalibrationMode::modes(M1_N_MODE, 1e-6),
        ]);
        let d: Vec<_> = recon
            .calib()
            .map(|c| c.mat_ref())
            .zip(
                m1_bm_recon
                    .calib()
                    .map(|c| (c.mat_ref(), c.mask_as_slice().to_vec())),
            )
            .map(|(c_txy, (c_bms, mask))| {
                let mut d = faer::Mat::<f64>::zeros(c_txy.nrows(), c_txy.ncols() + c_bms.ncols());
                d.as_mut()
                    .subcols_mut(0, c_txy.ncols())
                    .copy_from(c_txy * TXY_RESIDUAL_SCALING);
                d.as_mut()
                    .subcols_mut(c_txy.ncols(), c_bms.ncols())
                    .copy_from(c_bms);
                (d, mask)
            })
            .enumerate()
            .map(|(i, (d, mask))| {
                Calib::<MixedMirrorMode>::builder()
                    .c(d.col_iter()
                        .flat_map(|c| c.iter().copied())
                        .collect::<Vec<_>>())
                    .sid(i as u8 + 1)
                    .mask(mask)
                    .mode(mmode.clone())
                    .n_mode(M1_N_MODE + 2)
                    .build()
            })
            .collect();
        let mut recon = Reconstructor::<MixedMirrorMode>::new(d);
        recon
            .truncated_pseudoinverse(vec![2; 7])
            // .pseudoinverse()
            .to_data_repo("sh48_merged_recon.pkl")?;
        println!("{recon}");

        let sh48 = OpticalModelBuilder::from(
            &ShackHartmannBuilder::<Reconstructor<MixedMirrorMode>, R>::sh48()
                .use_calibration_src(),
        )
        .gmt(gmtb.clone())
        .build()?;

        let sh48_kern = Kernel::<Sh48MergerReconstructor<R>>::try_from(
            ShackHartmannBuilder::<Reconstructor<MixedMirrorMode>, R>::sh48().reconstructor(recon),
        )?
        .controller(Integrator::<Estimate>::new(M1_N_MODE * 7 + 42).gain(0.1));

        let merge_agws = MergeAgws::new();

        let timer: Timer = Timer::new(200);

        type Sh48Frame = KernelFrame<Sh48MergerReconstructor<R>>;

        actorscript!(
            #[model(name=agws)]
            #[labels(
                timer="⏲",
                sh24="GMT\nAGWS SH24",
                sh24_kern="AGWS SH24\nKernel",
                sh48="GMT\nAGWS SH48",
                sh48_kern="AGWS SH48\nKernel",
                on_axis="On-axis\nGMT",
                sh48_wave="SH48 GMT\nWave-Sensor",
                sampler = "1:10",
                sh24_frame = "SH24\nframe",
                sh48_frame = "SH48\nframe",
                optical_state_arrow = "Optics State\nLog")]
            1: timer[Tick] -> optical_state[OpticsState]
                -> sh24[Sh24Frame]! -> sh24_kern[M2RigidBodyMotions]
                    -> merge_agws[OpticsState] -> optical_state[OpticsState]..
                        -> on_axis[WfeRms<-9>] -> print
            1: optical_state[OpticsState].. -> optical_state_arrow
            1: optical_state[OpticsState] -> sh48
            10: sh48[Sh48Frame]! -> sh48_kern[Estimate] -> merge_agws
            10: sh48[Sh48Frame]!.. -> sh48_frame
            // 10: sh48_kern[SensorData]${48*48*6}
            1: sh24[Sh24Frame].. -> sampler
            10: sampler[Sh24Frame] -> sh24_frame
            1: on_axis[Wavefront] -> onaxis_wavefront_gif
            1: optical_state[OpticsState].. -> sh48_wave[Wavefront] -> sh48_wavefront_gif
            1: optical_state[M1State] -> m1[M1RBM<SID>].. -> m1_scopes
            1: m1[ModeShapes<SID>].. -> m1_scopes
            1: optical_state[M2State] -> m2[M2RBM<SID>].. -> m2_scopes
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
