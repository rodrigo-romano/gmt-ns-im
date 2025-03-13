use std::fs::File;

use faer::MatRef;
use gmt_dos_clients_crseo::{
    OpticalModelBuilder,
    calibration::{Calibration, CalibrationMode},
    centroiding::CentroidsProcessing,
    crseo::gmt::GmtM2,
};
use gmt_dos_systems_agws::builder::shack_hartmann::ShackHartmannBuilder;
use matio_rs::MatFile;
use skyangle::Conversion;

fn main() -> anyhow::Result<()> {
    let sh24 = ShackHartmannBuilder::<1>::sh24().use_calibration_src();
    let omb = OpticalModelBuilder::<_>::from(sh24);
    // dbg!(&omb);

    // Calibration of M2 Tz,Rx,Ry RBMs with AGWS SH24
    let mut recon = <CentroidsProcessing as Calibration<GmtM2>>::calibrate(
        &(omb.into()),
        CalibrationMode::RBM([
            None,
            None,
            #[cfg(feature = "pth")]
            None,
            #[cfg(feature = "rco")]
            Some(1e-6),
            Some(1f64.from_arcsec()),
            Some(1f64.from_arcsec()),
            None,
        ]),
    )?;
    recon.pseudoinverse();
    println!("{recon}");
    #[cfg(feature = "pth")]
    let mut file = File::create("recon_sh24-to-rbm_pth.pkl")?;
    #[cfg(feature = "rco")]
    let mut file = File::create("recon_sh24-to-rbm_rco.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;

    // Segment-wise multiplication of poke matrix pseudo-inverse
    // with [Tz,Rx,Ry]->(PZT actuators) matrix
    #[cfg(feature = "pth")]
    let matfile = MatFile::load("rbm_2_pzt_pth.mat")?;
    #[cfg(feature = "rco")]
    let matfile = MatFile::load("rbm_2_pzt_rco.mat")?;
    recon.pinv().enumerate().for_each(|(i, pinv)| {
        let var: Vec<f64> = matfile.var(format!("var{i}")).unwrap();
        #[cfg(feature = "pth")]
        let mat = MatRef::from_column_major_slice(&var, 3, 2);
        #[cfg(feature = "rco")]
        let mat = MatRef::from_column_major_slice(&var, 3, 3);
        // dbg!(&mat.row(0));
        pinv.transform(|x| mat * x).reset_mode();
    });
    println!("{recon}");

    #[cfg(feature = "pth")]
    let mut file = File::create("recon_sh24-to-pzt_pth.pkl")?;
    #[cfg(feature = "rco")]
    let mut file = File::create("recon_sh24-to-pzt_rco.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;

    Ok(())
}
