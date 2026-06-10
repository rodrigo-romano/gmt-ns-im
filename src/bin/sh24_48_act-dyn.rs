use std::{
    env,
    fs::{self},
    path::Path,
};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    gif::{Frame, Gif},
    iir::IIRFilter,
    integrator::Integrator,
    leftright,
    operator::Operator,
    print::Print,
    sampler::Sampler,
    timer::Timer,
};
use gmt_dos_clients_crseo::{
    OpticalModel, OpticalModelBuilder,
    calibration::{MixedMirrorMode, Reconstructor},
    crseo::{FromBuilder, Gmt},
    sensors::{NoSensor, WaveSensor},
};
use gmt_dos_clients_io::{
    gmt_m1::M1ModeShapes,
    gmt_m2::M2RigidBodyMotions,
    optics::{Wavefront, WfeRms},
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

use gmt_ns_im::agws::{
    self, calibration as agws_calibration,
    differential_reconstructor::DifferentialStackedReconstructor,
};
use interface::{Left, Right, Tick, filing::Filing};

/*
There are 3 type of reconstructor each activated with a different feature:
 * merged (`feature=merge`)
 * stacked (`feature=stack`)
 * stacked (`feature=stack`)
 * differential stacked (`feature=diffstack`)
*/

#[cfg(any(feature = "merge", not(any(feature = "stack", feature = "diffstack"))))]
type Sh48ReconstructorKind = agws_calibration::Merge;
#[cfg(feature = "stack")]
type Sh48ReconstructorKind = agws_calibration::Stack;
#[cfg(feature = "diffstack")]
type Sh48ReconstructorKind = agws_calibration::DiffStack;
#[cfg(not(feature = "diffstack"))]
type K48 = agws::Sh48Reconstructor<{ config::agws::sh48::RATE }>;
#[cfg(feature = "diffstack")]
type K48 = agws::Sh48DiffReconstructor<{ config::agws::sh48::RATE }>;
type Sh48Calibration =
    agws_calibration::Sh48Calibration<Sh48ReconstructorKind, agws_calibration::M2Txy>;
#[cfg(not(feature = "diffstack"))]
const TXY_RESIDUAL_SCALING: f64 =
    <Sh48ReconstructorKind as agws_calibration::Sh48Reconstructor>::TXY_RESIDUAL_SCALING;

#[tokio::main]
// From qp crate folder use:
// cargo r -r --bin sh24_48_act-dyn --features gmt_dos-systems_agws/shk24 --features interface/serde-pickle
async fn main() -> anyhow::Result<()> {
    let data_repo = Path::new(&env::var("DATA_REPO")?)
        .join("main")
        .join("decoupled");
    fs::create_dir_all(&data_repo)?;
    unsafe {
        env::set_var("DATA_REPO", data_repo);
    }

    // let recon: Reconstructor = serde_pickle::from_reader(
    //     File::open("../calibrations/sh24/recon_sh24-to-rbm_pth.pkl")?,
    //     Default::default(),
    // )?;
    let recon = Reconstructor::from_path("calibrations/sh24/recon_sh24-to-rbm_pth.pkl")?;
    println!("{recon}");
    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);

    let sh24 = OpticalModelBuilder::from(
        &ShackHartmannBuilder::<Reconstructor>::sh24().use_calibration_src(),
    )
    .gmt(gmtb.clone())
    .build()?;

    // SH24 kernel (no controller, just the reconstructor)
    let sh24_kern = Kernel::<Sh24TT<1>>::try_from(
        ShackHartmannBuilder::<Reconstructor>::sh24().reconstructor(recon),
    )?;

    // // Double integrator IIR coefficients (segment TT controller)
    let b_coeffs = vec![-0.06997, -0.004859, 0.06511]; // Feed-forward coefficients
    let a_coeffs = vec![-2.0, 1.0]; // Feedback coefficients (excluding a[0]=1.0)
    // let b_coeffs = vec![-0.5]; // Feed-forward coefficients
    // let a_coeffs = vec![-1.0]; // Feedback coefficients (excluding a[0]=1.0)
    let n_u = 42; // Number of inputs (M2 RBM)
    let dint_ttc = IIRFilter::new(b_coeffs, a_coeffs, n_u);

    let fsm_cl_dyn = IIRFilter::new(
        vec![0.09763139, 0.19526277, 0.09763139], // Feed-forward coefficients
        vec![-0.94281206, 0.33333760],            // Feedback coefficients (excluding a[0]=1.0)
        42,
    ); // Number of inputs (M2 RBM)

    let _sh48_wave = OpticalModel::<WaveSensor>::builder()
        .gmt(gmtb.clone())
        .source(AgwsGuideStar::sh48())
        .sensor(
            OpticalModel::<NoSensor>::builder()
                .source(AgwsGuideStar::sh48())
                .into(),
        )
        .build()?;

    let mirror = MirrorState::default()
        .set_segment_state(
            1,
            SegmentState::rbms(&[100e-6, 0., 0., 4.0e-5, -4.0e-5, 0.]),
        )
        .set_segment_state(2, SegmentState::rbms(&[0.0 * 1e-5, 0., 0., 0., 0., 0.]));
    let optical_state = OpticalState::default().set_zero_point(OpticalState::m2(mirror));
    // let optical_state = OpticalState::default()
    //     .set_zero_point(OpticalState::m2(mirror))
    //     .set_zero_point(OpticalState::m1(MirrorState::default()
    //         .set_segment_state(
    //             1,
    //             SegmentState::modes(vec![0f64; config::m1::segment::N_MODE]).set_mode(0, 1e-5)/,
    //     ),
    // ));

    // let optical_state = OpticalState::default().zero_point(OpticalState::m1(mirror));
    // let optical_state = OpticalState::default().zero_point(OpticalState::new(
    //     MirrorState::default().set_segment_state(
    //         1,
    //         SegmentState::modes(vec![0f64; config::m1::segment::N_MODE]).set_mode(1, 1e-6),
    //     ),
    //     mirror,
    // ));
    let optical_state_arrow = OpticalStateArrow::<M1State, M2RigidBodyMotions>::builder()
        .file_name(format!(
            "optical_state_{}m1-mode.parquet",
            config::m1::segment::N_MODE
        ))
        .build(config::m1::segment::N_MODE);

    let print = Print::default().tag("WFE RMS [nm]");

    let timer: Timer = Timer::new(200 * 5);

    let on_axis = OpticalModel::<NoSensor>::builder()
        .gmt(gmtb.clone())
        .build()?;

    let sh24_frame = Frame::<f32>::new("sh24-frame.gif", 24 * 12);
    let sampler: Sampler<_, _, M2State> = Sampler::default();

    let onaxis_wavefront_gif = Gif::new("on-axis_wavefront.gif", 512, 512)
        .expect("REASON")
        .delay(50);
    //let sh48_wavefront_gif = Gif::new("sh48_wavefront.gif", 512 * 3, 512)
    //    .expect("REASON")
    //    .delay(50 * 5);

    let m1_state = MirrorState::default();
    // let _m1_scopes = M1RBMScope::new()?;
    let m2_state = MirrorState::default();
    //let m2_scopes = M2RBMScope::new()?;

    type Sh24Frame = KernelFrame<Sh24TT<1>>;
    // type AgwsSh24 = Sh24<1>;
    // type AgwsSh24Kernel = Sh24Kern<Sh24TT<1>>;

    actorscript!(
        #[model(name=agws_sh24)]
        #[labels(
            timer="⏲",
            sh24="AGWS SH24",
            sh24_kern="AGWS SH24\nkernel",
            dint_ttc="TT feedback\ncontroller",
            fsm_cl_dyn="FSM CL\n dynamics",
            on_axis="On-axis\nGMT",
            sampler="One sample\ndelay",
            //sh48_wave="SH48 GMT\nWFS",
            sh24_frame = "SH24\nframe")]
        1: timer[Tick] -> optical_state[OpticsState] -> sh24
        1: on_axis[Wavefront] -> onaxis_wavefront_gif
        //1: optical_state[OpticsState] -> optical_state_arrow
        5: sh24[Sh24Frame]! -> sh24_kern[M2RigidBodyMotions]
            -> dint_ttc[M2RigidBodyMotions] -> fsm_cl_dyn[M2RigidBodyMotions] -> m2_state
        /* ---
        Choose one of the two line below. The first introduces a sample delay between
        the M2 state and the optical state, while the second assumes no delay (i.e., the
        optical state is updated immediately after the M2 state is updated).
        */
        1: m2_state[M2State] -> sampler[M2State]! -> optical_state
        //5: m2_state[M2State] -> sampler[M2State] -> optical_state
        // ---
        5: sh24[Sh24Frame] -> sh24_frame
        //5: optical_state[OpticsState] -> sh48_wave[Wavefront] -> sh48_wavefront_gif
        //1: optical_state[M1State] -> m1[M1RBM<1>].. -> m1_scopes
        //5: optical_state[M2State] -> m2[M2RBM<1>]${6}//~
        5: optical_state[OpticsState] -> on_axis[WfeRms<-9>] -> print
    );

    /*
     * Calibration of SH48 and M1 bending modes, and merging of the two calibrations
     * into a single reconstructor with both Sx Txy and M1 bending modes.
     * The merged reconstructor is then used to build the SH48 kernel.*/

    {
        use gmt_dos_clients_crseo::OpticalModelBuilder;
        use gmt_dos_clients_io::Estimate;
        //use gmt_dos_systems_agws::agws::sh48::Sh48;

        // const R: usize = 1000; //200 ms
        println!(
            "Using R={} for the SH48 reconstructor.",
            config::agws::sh48::RATE
        );

        let m2p_cl_dyn = IIRFilter::new(
            vec![0.00024136, 0.00048272, 0.00024136], // Feed-forward coefficients
            vec![-1.95557865, 0.95654408],            // Feedback coefficients (excluding a[0]=1.0)
            42,                                       //42,
        ); // Number of inputs (M2 RBM)
        println!("M2 POS CL dynamics sampled at {} Hz", 1000 / 5);

        let m1_cl_dyn = IIRFilter::new(
            vec![0.00024136, 0.00048272, 0.00024136], // Feed-forward coefficients
            vec![-1.95557865, 0.95654408],            // Feedback coefficients (excluding a[0]=1.0)
            config::m1::segment::N_MODE * 7,
        ); // Number of inputs (M2 RBM)
        println!("M1 CL dynamics sampled at {} Hz", 1000 / 5);

        let sh48 = OpticalModelBuilder::from(
            &ShackHartmannBuilder::<Reconstructor<MixedMirrorMode>, {config::agws::sh48::RATE}>::sh48()
                .use_calibration_src(),
        )
        .gmt(gmtb.clone())
        .build()?;

        let mut recon = Sh48Calibration::new()?
            .m1_modes(config::m1::segment::MODES, config::m1::segment::N_MODE)?
            .recon()?;
        #[cfg(not(feature = "diffstack"))]
        recon.pinv_iter_mut().for_each(|pinv| {
            pinv.transform(|mat| {
                // 1. Clone the entire matrix into a mutable owned matrix once
                let mut owned_mat = mat.to_owned();
                {
                    // 2. Extract a mutable slice of just the first 2 rows
                    let mut txy_rows = owned_mat.as_mut().subrows_mut(0, 2);
                    // 3. Multiply only these two rows in place using faer's optimized math
                    txy_rows.copy_from(&(txy_rows.as_ref() * TXY_RESIDUAL_SCALING));
                } // Block ends here to drop the temporary mutable borrow
                // 4. Return the updated matrix
                owned_mat
            });
        });
        #[cfg(not(feature = "diffstack"))]
        type RECON = Reconstructor<MixedMirrorMode>;
        #[cfg(feature = "diffstack")]
        type RECON = DifferentialStackedReconstructor;
        let sh48_kern = Kernel::<K48>::try_from(
            ShackHartmannBuilder::<RECON, { config::agws::sh48::RATE }>::sh48()
                .reconstructor(recon),
        )?
        .controller(Integrator::<Estimate>::new(config::m1::segment::N_MODE * 7 + 42).gain(0.9));

        type Sh48Frame = KernelFrame<K48>;

        // ===============================
        // -- SH48 ESTIMATE SPLITTER --
        // Split the SH48 command vector between M2 RBMS and M1 bending modes coefficients
        // The M2 RBMs are all zeros expect for Tx and Ty
        // The command vector `c` is arranged segment wise i.e `c=[c1,c2,c3,c4,c5,c6,c7]`
        // and each `ci` is the concantenation of the 6 M2 segment RBMS and the M1 bending modes
        let split = leftright::LeftRight::<Estimate, leftright::Split>::split_chunks_at(
            6 + config::m1::segment::N_MODE,
            6,
        );
        // ===============================
        // -- M2 SH48 RBMS (TXY) and SH24 (RXY) adder --
        let add_m2_rbms = Operator::plus();
        // ===============================

        let timer: Timer = Timer::new(3500); //200

        actorscript!(
            #[model(name=agws_sh24_48)]
            #[labels(
                timer="⏲",
                sh24="AGWS SH24",
                sh24_kern="AGWS SH24\nKernel",
                dint_ttc="TT feedback\nController",
                fsm_cl_dyn="FSM CL\nDynamics",
                m2p_cl_dyn="M2 POS CL\nDynamics",
                m1_cl_dyn="M1 Actuator\nDynamics",
                //merge_agws="Optical State\nMerger",
                sh48="AGWS SH48",
                sh48_kern="AGWS SH48\nKernel",
                split="Split AcO rec\ninto M2 RBM and\nM1 Shape coeffs",
                add_m2_rbms="+",
                on_axis="On-axis\nGMT",
                //sh48_wave="SH48 GMT\nWFS",
                sh24_frame = "SH24\nFrame",
                optical_state_arrow = "Optics State\nLog")]
            1: timer[Tick] -> optical_state[OpticsState] -> sh24
            1: optical_state[OpticsState] -> on_axis//[Wavefront] -> onaxis_wavefront_gif
            1: optical_state[OpticsState] -> sh48
            5: sh24[Sh24Frame]!
                -> sh24_kern[M2RigidBodyMotions]
                    -> dint_ttc[M2RigidBodyMotions]
                        -> fsm_cl_dyn[Right<M2RigidBodyMotions>]
                            -> add_m2_rbms
            //5: sh24[Sh24Frame]! -> sh24_kern[M2RigidBodyMotions] -> dint_ttc[M2RigidBodyMotions]
            //    -> fsm_cl_dyn[Right<M2RigidBodyMotions>] -> add_m2_rbms[M2RigidBodyMotions] -> merge_agws
            1000: sh48[Sh48Frame]! -> sh48_kern[Estimate]${105} -> split
            5: split[Left<Estimate>]
                -> m2p_cl_dyn[Left<Estimate>]
                    -> add_m2_rbms[M2RigidBodyMotions]${42}
                        -> m2_state
            5: split[Right<Estimate>] -> m1_cl_dyn[M1ModeShapes] -> m1_state
            // Log/Debug info
            1: m2_state[M2State] -> optical_state[OpticsState]
            1: m1_state[M1State] -> optical_state[OpticsState]! -> optical_state_arrow
            // 5: optical_state[OpticsState] -> sh48_wave[Wavefront] -> sh48_wavefront_gif
            // 5: m1_state[M1RBM<1>].. -> m1_scopes
            5: on_axis[WfeRms<-9>] -> print
            // R: sh48_kern[SensorData]${48*48*6}
        );

        /* No need to lock the scopes here, as they are not used in the main loop. If we
        wanted to use them, we would need to lock them before the select! macro, and then
        we would also need to handle the case where the scopes are closed (e.g., by breaking
        the loop). */
        /*
        // let mut m1_scopes_lock = m1_scopes.lock().await;
        let mut m2_scopes_lock = m2_scopes.lock().await;
        println!("Running... press Ctrl-C to stop");
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("Caught Ctrl-C, shutting down.");
            }
            // _ = m1_scopes_lock.close() => {}
            _ = m2_scopes_lock.close() => {}
        }
         */
    }

    Ok(())
}
