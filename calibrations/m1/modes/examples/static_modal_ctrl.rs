use std::fs::File;

use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::ExponentialMatrix};
use gmt_dos_clients_io::gmt_fem::{inputs::M1ActuatorsSegment1, outputs::M1Segment1AxialD};
use gmt_dos_systems_m1::SingularModes;
use gmt_ns_im::m1_bending_modes::M1BendingModes;
use matio_rs::MatFile;
use nalgebra as na;

fn main() -> anyhow::Result<()> {
    // let matfile = MatFile::load("20230530_1756_m1_mode_to_force.mat")?;
    // let b2f: na::DMatrix<f64> = matfile.var("B2F_1")?;
    // dbg!(&b2f.shape());

    // let s2b: Vec<na::DMatrix<f64>> = M1BendingModes::new("m1_singular_modes.pkl")?.into();
    // dbg!(s2b[0].shape());

    let m1_sms: SingularModes =
        serde_pickle::from_reader(&File::open("m1_singular_modes.pkl")?, Default::default())?;

    let m1_s1_sms = &m1_sms[0];

    let b2f = m1_s1_sms.mode2force();
    dbg!(&b2f.shape());

    let s2b = m1_s1_sms.raw_modes_into_mat().transpose();
    dbg!(s2b.shape());
    let s2bb = m1_s1_sms.modes_into_mat().transpose();
    dbg!(s2bb.shape());

    let static_gain = DiscreteModalSolver::<ExponentialMatrix>::from_env()?
        .ins::<M1ActuatorsSegment1>()
        .outs::<M1Segment1AxialD>()
        .static_gain()
        .unwrap();
    dbg!(&static_gain.shape());

    MatFile::save("static_modal.mat")?
        .var("b2f", &b2f)?
        .var("s2b", &s2b)?
        .var("s2bb", &s2bb)?
        .var("gain", &static_gain)?;

    let static_gain = DiscreteModalSolver::<ExponentialMatrix>::from_env()?
        .ins_with::<M1ActuatorsSegment1>(b2f.as_view())
        .outs_with::<M1Segment1AxialD>(s2bb.as_view())
        .static_gain()
        .unwrap();
    dbg!(&static_gain.shape());

    let mut b_in = vec![0f64; 329];
    b_in[5] = 1e-6;
    let b_out = &static_gain * na::DVector::from_column_slice(&b_in);
    dbg!(&b_out.column(0).as_slice()[..10]);

    Ok(())
}
