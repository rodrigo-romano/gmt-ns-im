use gmt_dos_clients_crseo::{
    calibration::{Calibration, CalibrationMode},
    centroiding::CentroidsProcessing,
    crseo::gmt::GmtM2,
};
use gmt_dos_systems_agws::{Agws, builder::AgwsGuideStar};
use skyangle::Conversion;

fn main() -> anyhow::Result<()> {
    let omb = Agws::<1, 1>::builder()
        .sh24()
        .source(AgwsGuideStar::sh24().fwhm(12f64));
    dbg!(&omb);
    let mut recon = <CentroidsProcessing as Calibration<GmtM2>>::calibrate(
        &(omb.into()),
        CalibrationMode::RBM([
            None,
            None,
            Some(1e-6),
            Some(1f64.from_arcsec()),
            Some(1f64.from_arcsec()),
            None,
        ]),
    )?;
    recon.pseudoinverse();
    println!("{recon}");
    Ok(())
}
