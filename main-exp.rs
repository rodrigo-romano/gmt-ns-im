/*
Model BOOTSTRAP [completed] :
  1: timer_actor[Tick] -> servos_clone
  1: mount_cmd_actor[MountSetPoint] -> servos_clone
  1: servos_clone[Doublet < M1State, M2State >] -> gmt_state_tx_actor
 */
/*
Model MODEL [completed] :
  1: timer_actor[Tick] -> servos_clone
  1: servos_clone[Doublet < M1State, M2State >] -> agws_clone
  1: servos_clone[Doublet < M1State, M2State >] -> agws_clone
  1: servos_clone[Doublet < M1State, M2State >] -> gmt_state_tx_actor
  1: servos_clone[M2PositionerNodes]
  1: servos_clone[M2FSMPiezoNodes] -> pzt_to_rbm_actor[M2RigidBodyMotions] -> pzt_to_rbm_int_actor[M2RigidBodyMotions] -> servos_clone
  1: servos_clone[M1EdgeSensors] -> m1_es_to_rbm_int_actor[M1RigidBodyMotions] -> servos_clone
  5: agws_clone[M2FSMFsmCommand] -> fsm_pzt_int_actor
  1: fsm_pzt_int_actor[M2FSMFsmCommand] -> servos_clone
5000: agws_clone[Estimate] -> sh48_int_actor[M1ModeShapes] -> m1_bm_2_forces_actor
  1: m1_bm_2_forces_actor[M1ActuatorCommandForces] -> servos_clone
 */
#![feature(prelude_import)]
#[prelude_import]
use std::prelude::rust_2024::*;
#[macro_use]
extern crate std;
use std::{env, fs::File, time::Instant};
use faer::{Mat, MatRef};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{
    gain::Gain, integrator::Integrator, low_pass_filter::LowPassFilter,
    operator::{Left, Operator, Right},
    signals::Signals, timer::Timer,
};
use gmt_dos_clients_crseo::{
    OpticalModel, calibration::{ClosedLoopCalib, Reconstructor},
    crseo::{FromBuilder, Gmt, builders::AtmosphereBuilder},
    sensors::NoSensor,
};
use gmt_dos_clients_io::{
    Estimate,
    gmt_m1::{
        M1EdgeSensors, M1ModeShapes, M1RigidBodyMotions,
        assembly::M1ActuatorCommandForces,
    },
    gmt_m2::{M2RigidBodyMotions, fsm::{M2FSMFsmCommand, M2FSMPiezoNodes}},
    mount::MountSetPoint, optics::{M1State, M2State},
};
use gmt_dos_clients_servos::{
    EdgeSensors, GmtFem, GmtM1, GmtM2, GmtM2Hex, GmtMount, GmtServoMechanisms,
    M1SegmentFigure,
};
use gmt_dos_clients_transceiver::{Monitor, Transceiver};
use gmt_dos_systems_agws::{
    Agws, agws::{sh24::Sh24, sh48::Sh48},
    builder::shack_hartmann::ShackHartmannBuilder, kernels::Kernel,
};
use gmt_fem::FEM;
use gmt_ns_im::{config, m1_bending_modes::M1BendingModes};
use interface::{Tick, doublet::Doublet};
use matio_rs::MatFile;
fn main() -> anyhow::Result<()> {
    let body = async {
        env_logger::init();
        {
            ::std::io::_print(
                format_args!(
                    "FEM  : {0}\n",
                    "/home/ubuntu/mnt/20230530_1756_zen_30_M1_202110_FSM_202305_Mount_202305_noStairs",
                ),
            );
        };
        {
            ::std::io::_print(format_args!("MOUNT: {0}\n", "MOUNT_FDR_1kHz"));
        };
        let now = Instant::now();
        let sim_sampling_frequency = 1000;
        let sim_duration = 40_usize;
        let bootstrapping_duration = 4_usize;
        let n_bootstrapping = sim_sampling_frequency * bootstrapping_duration;
        let n_sim = n_bootstrapping + sim_sampling_frequency * sim_duration + 1;
        let fem = FEM::from_env()?;
        let m1_es_2_rbm: nalgebra::DMatrix<f64> = MatFile::load(
                "calibrations/m1/edge-sensors/es_2_rbm.mat",
            )?
            .var("m1_r_es")?;
        let servos = GmtServoMechanisms::<
            { config::m1::segment::ACTUATOR_RATE },
            1,
        >::new(sim_sampling_frequency as f64, fem)
            .edge_sensors(EdgeSensors::m1().m1_with(m1_es_2_rbm))
            .m1_segment_figure(
                M1SegmentFigure::new()
                    .transforms(
                        M1BendingModes::new(
                                "calibrations/m1/modes/m1_singular_modes.pkl",
                            )?
                            .into(),
                    ),
            )
            .build()?;
        {
            ::std::io::_print(format_args!("{0}\n", servos));
        };
        let recon: Reconstructor = serde_pickle::from_reader(
            File::open("calibrations/sh24/recon_sh24-to-pzt_pth.pkl")?,
            Default::default(),
        )?;
        let m1_bm_recon: Reconstructor<_, ClosedLoopCalib> = serde_pickle::from_reader(
            File::open("calibrations/sh48/closed_loop_recon_sh48-to-m1-bm.pkl")?,
            Default::default(),
        )?;
        {
            ::std::io::_print(format_args!("SH24 to FSM reconstructor:\n{0}\n", recon));
        };
        let (
            agws_wss,
            mut agws,
        ): (_, Sys<Agws<{ config::agws::sh48::RATE }, { config::agws::sh24::RATE }>>) = {
            let agws = if config::ATMOSPHERE {
                Agws::builder()
                    .load_atmosphere(
                        "atmosphere/atmosphere.toml",
                        sim_sampling_frequency as f64,
                    )?
            } else {
                Agws::builder()
                    .sh24(ShackHartmannBuilder::sh24().use_calibration_src())
                    .sh48(ShackHartmannBuilder::sh48().use_calibration_src())
            }
                .gmt(
                    Gmt::builder()
                        .m1(
                            gmt_ns_im::config::m1::segment::RAW_MODES,
                            gmt_ns_im::config::m1::segment::N_RAW_MODE,
                        ),
                )
                .sh24_calibration(recon)
                .sh48_calibration(m1_bm_recon);
            (agws.wave_sensor().build()?, agws.build()?)
        };
        if let Some(p24) = config::agws::sh24::POINTING_ERROR {
            let _ = agws.sh24_pointing(p24).await;
        }
        {
            ::std::io::_print(format_args!("{0}\n", agws));
        };
        {
            ::std::io::_print(format_args!("{0}\n", agws_wss));
        };
        let fsm_pzt_int = Integrator::new(21).gain(config::agws::sh24::INTEGRATOR_GAIN);
        let matfile = MatFile::load("calibrations/sh24/m2_pzt_r.mat")?;
        let pzt_to_rbm: Vec<Mat<f64>> = (0..7)
            .map(|i| {
                let var: Vec<f64> = matfile
                    .var(
                        ::alloc::__export::must_use({
                            ::alloc::fmt::format(format_args!("var{0}", i))
                        }),
                    )
                    .unwrap();
                let mat = MatRef::from_column_major_slice(&var, 6, 6);
                mat.to_owned()
            })
            .collect();
        let pzt_to_rbm = Gain::<f64>::new(pzt_to_rbm);
        let pzt_to_rbm_int = Integrator::new(42)
            .gain(config::fsm::OFFLOAD_INTEGRATOR_GAIN);
        let m1_es_to_rbm_int = Integrator::new(42)
            .gain(config::m1::edge_sensor::RBM_INTEGRATOR_GAIN);
        {
            ::std::io::_print(
                format_args!("Model built in {0}s\n", now.elapsed().as_secs()),
            );
        };
        let mount_cmd = Signals::new(3, n_sim);
        let mut m1_rbm = ::alloc::vec::from_elem(::alloc::vec::from_elem(0f64, 6), 7);
        m1_rbm[0][0] = 1e-6;
        m1_rbm[6][4] = 1e-6;
        let m1_rbm = Signals::from((m1_rbm, n_sim));
        let mut m2_rbm = ::alloc::vec::from_elem(::alloc::vec::from_elem(0f64, 6), 7);
        m2_rbm[0][0] = 1e-6;
        m2_rbm[0][3] = 1e-6;
        m2_rbm[1][2] = 1e-6;
        m2_rbm[1][2] = 1e-6;
        m2_rbm[4][1] = 1e-6;
        m2_rbm[6][1] = 1e-6;
        let m2_rbm = Signals::from((m2_rbm, n_sim));
        let m2_adder = Operator::<f64>::new("+");
        let matfile = MatFile::load(
            "calibrations/m1/modes/20230530_1756_m1_mode_to_force.mat",
        )?;
        let b2f: Vec<Mat<f64>> = (1..=7)
            .map(|i| {
                matfile
                    .var(
                        ::alloc::__export::must_use({
                            ::alloc::fmt::format(format_args!("B2F_{0}", i))
                        }),
                    )
                    .unwrap()
            })
            .collect();
        {
            ::std::io::_print(
                format_args!(
                    "B2F: {0:?}\n",
                    b2f.iter().map(|x| x.shape()).collect::<Vec<_>>(),
                ),
            );
        };
        let m1_bm_2_forces = Gain::<
            f64,
        >::new(
            b2f
                .iter()
                .map(|x| x.subcols(0, config::m1::segment::N_MODE).to_owned())
                .collect::<Vec<_>>(),
        );
        let timer: Timer = Timer::new(n_sim);
        let address = "127.0.0.1";
        let mut gmt_state_mon = Monitor::new();
        let gmt_state_tx = Transceiver::<
            Doublet<M1State, M2State>,
        >::transmitter(address)?
            .run(&mut gmt_state_mon);
        let mut servos_clone = servos.clone();
        let mut timer = ::gmt_dos_actors::client::Client::from(timer);
        let mut mount_cmd = ::gmt_dos_actors::client::Client::from(mount_cmd);
        let mut gmt_state_tx = ::gmt_dos_actors::client::Client::from(gmt_state_tx);
        let mut timer_actor: ::gmt_dos_actors::prelude::Actor<_, 0, 1> = ::gmt_dos_actors::prelude::Actor::from(
            &timer,
        );
        let mut mount_cmd_actor: ::gmt_dos_actors::prelude::Actor<_, 0, 1> = ::gmt_dos_actors::prelude::Actor::from(
            &mount_cmd,
        );
        let mut gmt_state_tx_actor: ::gmt_dos_actors::prelude::Actor<_, 1, 0> = ::gmt_dos_actors::prelude::Actor::from(
            &gmt_state_tx,
        );
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::add_output(
            &mut timer_actor,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            Tick,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input::<
            GmtFem,
        >(output, &mut servos_clone)?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::add_output(
            &mut mount_cmd_actor,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            MountSetPoint,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input::<
            GmtMount,
        >(output, &mut servos_clone)?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::<
            GmtFem,
            1usize,
            1usize,
        >::add_output(&mut servos_clone);
        let actor_output = ::gmt_dos_actors::framework::network::AddOuput::unbounded(
            actor_output,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            Doublet<M1State, M2State>,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input(
            output,
            &mut gmt_state_tx_actor,
        )?;
        #[allow(unused_variables)]
        let bootstrap = ::gmt_dos_actors::model::Model::new(
                (<[_]>::into_vec(
                    ::alloc::boxed::box_new([
                        Box::new(servos_clone),
                        Box::new(timer_actor),
                        Box::new(mount_cmd_actor),
                        Box::new(gmt_state_tx_actor),
                    ]),
                )),
            )
            .name("bootstrap");
        let model = ::gmt_dos_actors::prelude::FlowChart::flowchart_open(bootstrap)
            .check()?
            .run()
            .await?;
        let sh48_m2_rbm_recon: Reconstructor = serde_pickle::from_reader(
            File::open("calibrations/sh48/open_loop_recon_sh48-to-m2-rbm.pkl")?,
            Default::default(),
        )?;
        {
            ::std::io::_print(
                format_args!("SH48 to M2 RBM reconstructor:\n{0}\n", sh48_m2_rbm_recon),
            );
        };
        let m1_bm_adder = Operator::<f64>::new("+");
        let sh48_int = Integrator::new(27 * 7).gain(0.5);
        let lpf = LowPassFilter::new(42, 2e-3);
        type AgwsSh48 = Sh48<{ config::agws::sh48::RATE }>;
        type AgwsSh24 = Sh24<{ config::agws::sh24::RATE }>;
        type AgwsSh24Kernel = Kernel<Sh24<{ config::agws::sh24::RATE }>>;
        type AgwsSh48Kernel = Kernel<Sh48<{ config::agws::sh48::RATE }>>;
        let mut pzt_to_rbm_int = ::gmt_dos_actors::client::Client::from(pzt_to_rbm_int);
        let mut servos_clone = servos.clone();
        let mut timer = ::gmt_dos_actors::client::Client::from(timer);
        let mut agws_clone = agws.clone();
        let mut fsm_pzt_int = ::gmt_dos_actors::client::Client::from(fsm_pzt_int);
        let mut gmt_state_tx = ::gmt_dos_actors::client::Client::from(gmt_state_tx);
        let mut m1_es_to_rbm_int = ::gmt_dos_actors::client::Client::from(
            m1_es_to_rbm_int,
        );
        let mut sh48_int = ::gmt_dos_actors::client::Client::from(sh48_int);
        let mut pzt_to_rbm = ::gmt_dos_actors::client::Client::from(pzt_to_rbm);
        let mut m1_bm_2_forces = ::gmt_dos_actors::client::Client::from(m1_bm_2_forces);
        mount_cmd.set_label("Mount Set-Point");
        m1_bm_2_forces.set_label("Mode to Force");
        fsm_pzt_int.set_label("FSM\nIntegrator");
        pzt_to_rbm.set_label("FSM\nto\nPositioner");
        pzt_to_rbm_int.set_label("Positioner\nIntegrator");
        m1_es_to_rbm_int.set_label("M1 RBM\nIntegrator");
        sh48_int.set_label("M1 BM\nIntegrator");
        gmt_state_tx.set_label("🕪");
        let mut pzt_to_rbm_int_actor: ::gmt_dos_actors::prelude::Actor<_, 1, 1> = ::gmt_dos_actors::prelude::Actor::from(
            &pzt_to_rbm_int,
        );
        let mut timer_actor: ::gmt_dos_actors::prelude::Actor<_, 0, 1> = ::gmt_dos_actors::prelude::Actor::from(
            &timer,
        );
        let mut fsm_pzt_int_actor: ::gmt_dos_actors::prelude::Actor<_, 5, 1> = ::gmt_dos_actors::prelude::Actor::from(
            &fsm_pzt_int,
        );
        let mut gmt_state_tx_actor: ::gmt_dos_actors::prelude::Actor<_, 1, 0> = ::gmt_dos_actors::prelude::Actor::from(
            &gmt_state_tx,
        );
        let mut m1_es_to_rbm_int_actor: ::gmt_dos_actors::prelude::Actor<_, 1, 1> = ::gmt_dos_actors::prelude::Actor::from(
            &m1_es_to_rbm_int,
        );
        let mut sh48_int_actor: ::gmt_dos_actors::prelude::Actor<_, 5000, 5000> = ::gmt_dos_actors::prelude::Actor::from(
            &sh48_int,
        );
        let mut pzt_to_rbm_actor: ::gmt_dos_actors::prelude::Actor<_, 1, 1> = ::gmt_dos_actors::prelude::Actor::from(
            &pzt_to_rbm,
        );
        let mut m1_bm_2_forces_actor: ::gmt_dos_actors::prelude::Actor<_, 5000, 1> = ::gmt_dos_actors::prelude::Actor::from(
            &m1_bm_2_forces,
        );
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::add_output(
            &mut timer_actor,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            Tick,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input::<
            GmtFem,
        >(output, &mut servos_clone)?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::<
            GmtFem,
            1usize,
            1usize,
        >::add_output(&mut servos_clone);
        let actor_output = ::gmt_dos_actors::framework::network::AddOuput::bootstrap(
            actor_output,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            Doublet<M1State, M2State>,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input::<
            AgwsSh24,
        >(output, &mut agws_clone)?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::<
            GmtFem,
            1usize,
            1usize,
        >::add_output(&mut servos_clone);
        let actor_output = ::gmt_dos_actors::framework::network::AddOuput::bootstrap(
            actor_output,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            Doublet<M1State, M2State>,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input::<
            AgwsSh48,
        >(output, &mut agws_clone)?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::<
            GmtFem,
            1usize,
            1usize,
        >::add_output(&mut servos_clone);
        let actor_output = ::gmt_dos_actors::framework::network::AddOuput::bootstrap(
            actor_output,
        );
        let actor_output = ::gmt_dos_actors::framework::network::AddOuput::unbounded(
            actor_output,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            Doublet<M1State, M2State>,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input(
            output,
            &mut gmt_state_tx_actor,
        )?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::<
            GmtFem,
            1usize,
            1usize,
        >::add_output(&mut servos_clone);
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            M2FSMPiezoNodes,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input(
            output,
            &mut pzt_to_rbm_actor,
        )?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::add_output(
            &mut pzt_to_rbm_actor,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            M2RigidBodyMotions,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input(
            output,
            &mut pzt_to_rbm_int_actor,
        )?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::add_output(
            &mut pzt_to_rbm_int_actor,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            M2RigidBodyMotions,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input::<
            GmtM2Hex,
        >(output, &mut servos_clone)?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::<
            GmtFem,
            1usize,
            1usize,
        >::add_output(&mut servos_clone);
        let actor_output = ::gmt_dos_actors::framework::network::AddOuput::bootstrap(
            actor_output,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            M1EdgeSensors,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input(
            output,
            &mut m1_es_to_rbm_int_actor,
        )?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::add_output(
            &mut m1_es_to_rbm_int_actor,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            M1RigidBodyMotions,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input::<
            GmtM1,
        >(output, &mut servos_clone)?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::<
            AgwsSh24Kernel,
            5usize,
            5usize,
        >::add_output(&mut agws_clone);
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            M2FSMFsmCommand,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input(
            output,
            &mut fsm_pzt_int_actor,
        )?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::add_output(
            &mut fsm_pzt_int_actor,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            M2FSMFsmCommand,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input::<
            GmtM2,
        >(output, &mut servos_clone)?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::<
            AgwsSh48Kernel,
            5000usize,
            5000usize,
        >::add_output(&mut agws_clone);
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            Estimate,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input(
            output,
            &mut sh48_int_actor,
        )?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::add_output(
            &mut sh48_int_actor,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            M1ModeShapes,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input(
            output,
            &mut m1_bm_2_forces_actor,
        )?;
        let actor_output = ::gmt_dos_actors::framework::network::AddActorOutput::add_output(
            &mut m1_bm_2_forces_actor,
        );
        let output = ::gmt_dos_actors::framework::network::AddOuput::build::<
            M1ActuatorCommandForces,
        >(actor_output);
        gmt_dos_actors::framework::network::TryIntoInputs::into_input::<
            GmtM1,
        >(output, &mut servos_clone)?;
        #[allow(unused_variables)]
        let model = ::gmt_dos_actors::model::Model::new(
                (<[_]>::into_vec(
                    ::alloc::boxed::box_new([
                        Box::new(pzt_to_rbm_int_actor),
                        Box::new(servos_clone),
                        Box::new(timer_actor),
                        Box::new(agws_clone),
                        Box::new(fsm_pzt_int_actor),
                        Box::new(gmt_state_tx_actor),
                        Box::new(m1_es_to_rbm_int_actor),
                        Box::new(sh48_int_actor),
                        Box::new(pzt_to_rbm_actor),
                        Box::new(m1_bm_2_forces_actor),
                    ]),
                )),
            )
            .name("model");
        let model = ::gmt_dos_actors::prelude::FlowChart::flowchart_open(model)
            .check()?
            .run()
            .await?;
        Ok(())
    };
    #[allow(
        clippy::expect_used,
        clippy::diverging_sub_expression,
        clippy::needless_return
    )]
    {
        return tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .expect("Failed building the Runtime")
            .block_on(body);
    }
}
