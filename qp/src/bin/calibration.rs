use std::{error::Error, fs::File, io::BufWriter};

use gmt_dos_clients_crseo::{
    OpticalModelBuilder,
    calibration::{
        Calibration, CalibrationMode, MirrorMode, Reconstructor,
        algebra::{Block, CalibProps},
    },
    centroiding::CentroidsProcessing,
    crseo::{
        FromBuilder, Gmt,
        gmt::{GmtM1, GmtM2},
    },
};
use gmt_dos_systems_agws::builder::shack_hartmann::ShackHartmannBuilder;

const M1_N_MODE: usize = 27;

fn main() -> Result<(), Box<dyn Error>> {
    let sh48 = ShackHartmannBuilder::<Reconstructor>::sh48().use_calibration_src();
    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);
    let omb = OpticalModelBuilder::from(sh48).gmt(gmtb);
    println!("{}", omb.clone().build()?);

    // M1 & M2 RBMs without S7 Rz
    let mut rbm = [Some(1e-6); 6];
    rbm[5] = None;
    let m12_rbms = MirrorMode::new([Some(CalibrationMode::rbm(1e-6)); 7])
        .update((7, CalibrationMode::RBM(rbm)));
    // M1 RBM
    let recon = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
        &(omb.clone().into()),
        m12_rbms.clone(),
    )?;
    let m1_rbm_recon = recon.diagonal();
    println!("{m1_rbm_recon}");
    // M1 BM
    let recon = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
        &(omb.clone().into()),
        CalibrationMode::modes(M1_N_MODE, 1e-6),
    )?;
    let m1_bm_recon = recon.diagonal();
    println!("{m1_bm_recon}");
    // M2 RBM
    let recon = <CentroidsProcessing as Calibration<GmtM2>>::calibrate(&(omb.into()), m12_rbms)?;
    let m2_recon = recon.diagonal();
    println!("{m2_recon}");

    let recon = Reconstructor::block(&[&[&m1_rbm_recon, &m2_recon, &m1_bm_recon]]);
    println!("{recon}");
    let mat: Vec<_> = recon.calib_slice()[0]
        .mat_ref()
        .col_iter()
        .flat_map(|c| c.iter().cloned().collect::<Vec<_>>())
        .collect();
    let file = File::create("sh48_calibration.pkl")?;
    let mut buffer = BufWriter::new(file);
    serde_pickle::to_writer(&mut buffer, &recon.calib_slice()[0], Default::default())?;
    Ok(())
}
