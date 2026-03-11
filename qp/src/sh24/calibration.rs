use gmt_dos_clients_crseo::{
    OpticalModelBuilder,
    calibration::{
        Calib, Calibration, CalibrationError, CalibrationMode, ClosedLoopCalibration,
        ClosedLoopReconstructor, MixedMirrorMode, Reconstructor, algebra::CalibProps,
    },
    centroiding::CentroidsProcessing,
    crseo::{
        Imaging,
        builders::GmtBuilder,
        gmt::{GmtM1, GmtM2},
    },
    sensors::builders::CameraBuilder,
};
use gmt_dos_systems_agws::builder::shack_hartmann::ShackHartmannBuilder;
use interface::filing::{Filing, FilingError};

use crate::sh24::TXY_RESIDUAL_SCALING;

#[derive(Debug, thiserror::Error)]
pub enum Sh48CalibrationError {
    #[error("failed to write reconstructor to data repository")]
    SaveRecon(#[from] FilingError),
    #[error("failed to calibration SH48")]
    Calibrate(#[from] CalibrationError),
}

pub struct Sh48Calibration {
    m2_txy: ClosedLoopReconstructor,
    m1_bm: Option<Reconstructor>,
    m1_n_mode: usize,
}
type Result<T> = std::result::Result<T, Sh48CalibrationError>;

impl Sh48Calibration {
    pub fn new() -> Result<Self> {
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
                recon.to_data_repo(file_name)?;
                recon
            };
        println!("{recon}");
        Ok(Self {
            m2_txy: recon,
            m1_bm: None,
            m1_n_mode: 0,
        })
    }
    pub fn m1_modes(mut self, m1_n_mode: usize, gmtb: GmtBuilder) -> Result<Self> {
        // calibration of M1 Sx bending modes with SH48
        let file_name = format!("sh48_{}_bending-modes_calib.pkl", m1_n_mode);
        let m1_bm_recon: Reconstructor =
            if let Ok(recon) = Reconstructor::from_data_repo(&file_name) {
                recon
            } else {
                let sh48_omb: OpticalModelBuilder<CameraBuilder<1>> =
                    (&ShackHartmannBuilder::<Reconstructor>::sh48().use_calibration_src()).into();
                let recon = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
                    &(&sh48_omb.gmt(gmtb.clone())).into(),
                    CalibrationMode::modes(m1_n_mode, 1e-6),
                )?;
                recon.to_data_repo(file_name)?;
                recon
            };
        println!("{m1_bm_recon}");
        Ok(Self {
            m1_bm: Some(m1_bm_recon),
            m1_n_mode,
            ..self
        })
    }
    pub fn recon(self) -> Result<Reconstructor<MixedMirrorMode>> {
        Ok(if let Some(m1_bm_recon) = self.m1_bm {
            let file_name = format!(
                "sh48_merged_m2-txy_{}_bending-modes_recon.pkl",
                self.m1_n_mode
            );
            if let Ok(recon) = Reconstructor::from_data_repo(&file_name) {
                recon
            } else {
                let mmode = MixedMirrorMode::from(vec![
                    CalibrationMode::t_xy(1e-6),
                    CalibrationMode::modes(self.m1_n_mode, 1e-6),
                ]);
                let d: Vec<_> = self
                    .m2_txy
                    .calib()
                    .map(|c| c.mat_ref())
                    .zip(
                        m1_bm_recon
                            .calib()
                            .map(|c| (c.mat_ref(), c.mask_as_slice().to_vec())),
                    )
                    .map(|(c_txy, (c_bms, mask))| {
                        let mut d =
                            faer::Mat::<f64>::zeros(c_txy.nrows(), c_txy.ncols() + c_bms.ncols());
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
                            .n_mode(self.m1_n_mode + 2)
                            .build()
                    })
                    .collect();
                let mut recon = Reconstructor::<MixedMirrorMode>::new(d);
                recon
                    .truncated_pseudoinverse(vec![2; 7])
                    // .pseudoinverse()
                    .to_data_repo(&file_name)?;
                println!("{recon}");
                recon
            }
        } else {
            let mut recon = Reconstructor::<MixedMirrorMode>::new(
                self.m2_txy
                    .calib()
                    .map(|c| c.m1_closed_loop_to_sensor().to_owned().into())
                    .collect(),
            );
            recon.pseudoinverse();
            recon
        })
    }
}

pub fn sh48_calibration(
    gmtb: GmtBuilder,
    m1_n_mode: usize,
) -> std::result::Result<Reconstructor<MixedMirrorMode>, anyhow::Error> {
    // closed-loop calibration of M2 Sx Txy with SH48
    let file_name = "sh48_closed-loop_Txy_calib.pkl";
    let recon: ClosedLoopReconstructor = if let Ok(recon) =
        ClosedLoopReconstructor::from_data_repo(file_name)
    {
        recon
    } else {
        let sh48_omb: OpticalModelBuilder<CameraBuilder<1>> =
            (&ShackHartmannBuilder::<Reconstructor>::sh48().use_calibration_src()).into();
        let sh24_omb: OpticalModelBuilder<CameraBuilder<1>> =
            (&ShackHartmannBuilder::<Reconstructor>::sh24().use_calibration_src()).into();
        let mut recon = <CentroidsProcessing as ClosedLoopCalibration<GmtM2, Imaging>>::calibrate(
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
    let file_name = format!("sh48_{}_bending-modes_calib.pkl", m1_n_mode);
    let m1_bm_recon: Reconstructor = if let Ok(recon) = Reconstructor::from_data_repo(&file_name) {
        recon
    } else {
        let sh48_omb: OpticalModelBuilder<CameraBuilder<1>> =
            (&ShackHartmannBuilder::<Reconstructor>::sh48().use_calibration_src()).into();
        let mut recon = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
            &(&sh48_omb.gmt(gmtb.clone())).into(),
            CalibrationMode::modes(m1_n_mode, 1e-6),
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
        CalibrationMode::modes(m1_n_mode, 1e-6),
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
                .n_mode(m1_n_mode + 2)
                .build()
        })
        .collect();
    let mut recon = Reconstructor::<MixedMirrorMode>::new(d);
    recon
        .truncated_pseudoinverse(vec![2; 7])
        // .pseudoinverse()
        .to_data_repo("sh48_merged_recon.pkl")?;
    println!("{recon}");
    Ok(recon)
}
