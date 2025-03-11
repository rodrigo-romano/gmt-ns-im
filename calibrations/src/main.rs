use std::fs::File;

use faer::MatRef;
use gmt_dos_clients_crseo::{
    OpticalModelBuilder,
    calibration::{Calibration, CalibrationMode},
    centroiding::CentroidsProcessing,
    crseo::{
        FromBuilder,
        gmt::GmtM2,
        imaging::{Detector, LensletArray},
    },
    sensors::Camera,
};
use gmt_dos_systems_agws::{
    Agws,
    builder::{AgwsGuideStar, shack_hartmann::ShackHartmannBuilder},
};
use matio_rs::MatFile;
use skyangle::Conversion;

fn main() -> anyhow::Result<()> {
    // let omb = Agws::<1, 1>::builder()
    //     .sh24()
    //     // calibration source with FWHM roughly the size of the seeing
    //     .source(AgwsGuideStar::sh24().fwhm(12f64));
    // let sh24 = Agws::<1, 1>::builder()
    //     .sh24()
    //     // calibration source with FWHM roughly the size of the seeing
    //     .source(AgwsGuideStar::sh24().fwhm(12.));
    let sh24 = ShackHartmannBuilder::<1>::new().sensor(
        Camera::builder()
            .lenslet_array(LensletArray::default().n_side_lenslet(24).n_px_lenslet(36))
            .detector(Detector::default().n_px_framelet(36))
            .lenslet_flux(0.75),
    );
    let omb = OpticalModelBuilder::<_>::from(sh24);
    // dbg!(&omb);

    // Calibration of M2 Tz,Rx,Ry RBMs with AGWS SH24
    let mut recon = <CentroidsProcessing as Calibration<GmtM2>>::calibrate(
        &(omb.into()),
        CalibrationMode::RBM([
            None,
            None,
            None,
            // Some(1e-6),
            Some(1f64.from_arcsec()),
            Some(1f64.from_arcsec()),
            None,
        ]),
    )?;
    recon.pseudoinverse();
    println!("{recon}");
    let mut file = File::create("recon_sh24-to-rbm.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;

    // Segment-wise multiplication of poke matrix pseudo-inverse
    // with [Tz,Rx,Ry]->(PZT actuators) matrix
    let matfile = MatFile::load("rbm_2_pzt.mat")?;
    recon.pinv().enumerate().for_each(|(i, pinv)| {
        let var: Vec<f64> = matfile.var(format!("var{i}")).unwrap();
        let mat = MatRef::from_column_major_slice(&var, 3, 2);
        dbg!(&mat.row(0));
        pinv.transform(|x| mat * x).reset_mode();
    });
    println!("{recon}");

    let mut file = File::create("recon_sh24-to-pzt.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;

    Ok(())
}
