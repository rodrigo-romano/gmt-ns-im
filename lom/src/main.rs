use std::path::Path;

use gmt_lom::{LOM, OpticalMetrics, RigidBodyMotions};

fn main() -> anyhow::Result<()> {
    let path =
        Path::new("/home/ubuntu/projects/gmt-ns-im/web_server/static/main/optical_state.parquet");
    let rbm = RigidBodyMotions::from_parquet(
        path,
        Some("M1RigidBodyMotions"),
        Some("M2RigidBodyMotions"),
    )?;
    let mut lom = LOM::builder().build()?;
    lom.rbm = rbm.clone();
    let segment_piston = lom.segment_piston();
    let sp: Vec<_> = segment_piston.items().last().unwrap().iter().map(|x| *x*1e9).collect();
    println!("GMT segment piston: {sp:+4.0?} nm");

    let mut m1_rbm = rbm.clone();
    m1_rbm.zeroed_m2();
    lom.rbm = m1_rbm;
    let segment_piston = lom.segment_piston();
    let sp: Vec<_> = segment_piston.items().last().unwrap().iter().map(|x| *x*1e9).collect();
    println!("M1 segment piston : {sp:+4.0?} nm");

    let mut m2_rbm = rbm.clone();
    m2_rbm.zeroed_m1();
    lom.rbm = m2_rbm;
    let segment_piston = lom.segment_piston();
    let sp: Vec<_> = segment_piston.items().last().unwrap().iter().map(|x| *x*1e9).collect();
    println!("M2 segment piston : {sp:+4.0?} nm");

    Ok(())
}
