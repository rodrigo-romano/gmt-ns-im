fn main() -> anyhow::Result<()> {
    #[cfg(any(feature = "all", feature = "closed_loop", feature = "open_loop"))]
    {
        #[cfg(feature = "closed_loop")]
        {
            #[cfg(feature = "m1_rbm")]
            calibrations_sh48::closed_loop::m1_rbm()?;
            #[cfg(feature = "m2_rbm")]
            calibrations_sh48::closed_loop::m2_rbm()?;
            #[cfg(feature = "m1_bm")]
            calibrations_sh48::closed_loop::m1_bm()?;
        }
        #[cfg(feature = "open_loop")]
        {
            #[cfg(feature = "m1_rbm")]
            calibrations_sh48::open_loop::m1_rbm()?;
            #[cfg(feature = "m2_rbm")]
            calibrations_sh48::open_loop::m2_rbm()?;
            #[cfg(feature = "m1_bm")]
            calibrations_sh48::open_loop::m1_bm()?;
        }
    }
    Ok(())
}
/* fn main() -> anyhow::Result<()> {
    let sh48 = ShackHartmannBuilder::<1>::sh48().use_calibration_src();
    let omb = OpticalModelBuilder::<_>::from(sh48)
        .gmt(Gmt::builder().m1(gmt_ns_im::config::m1::segment::MODES, gmt_ns_im::config::m1::segment::N_MODE));
    dbg!(&omb);

    // Calibration of M1 RBMs with AGWS SH48
    let mut recon = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
        &(omb.clone().into()),
        CalibrationMode::RBM([
            Some(1e-6),
            Some(1e-6),
            Some(1e-6),
            Some(1f64.from_arcsec()),
            Some(1f64.from_arcsec()),
            Some(1f64.from_arcsec()),
        ]),
    )?;
    recon.pseudoinverse();
    println!("{recon}");
    let mut file = File::create("recon_sh48-to-m1-rbm.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;

    // Calibration of M1 BMs with AGWS SH48
    let mut recon = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
        &(omb.clone().into()),
        CalibrationMode::modes(gmt_ns_im::config::m1::segment::N_MODE, 1e-6),
    )?;
    recon.pseudoinverse();
    println!("{recon}");
    let mut file = File::create("recon_sh48-to-m1-bm.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;

    // Calibration of M2 RBMs with AGWS SH48
    let mut recon = <CentroidsProcessing as Calibration<GmtM2>>::calibrate(
        &(omb.into()),
        CalibrationMode::RBM([
            Some(1e-6),
            Some(1e-6),
            Some(1e-6),
            Some(1f64.from_arcsec()),
            Some(1f64.from_arcsec()),
            Some(1f64.from_arcsec()),
        ]),
    )?;
    recon.pseudoinverse();
    println!("{recon}");
    let mut file = File::create("recon_sh48-to-m2-rbm.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;

    Ok(())
} */
