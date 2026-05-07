use std::marker::PhantomData;

use gmt_dos_clients_crseo::{
    OpticalModelBuilder,
    calibration::{
        Calib, Calibration, CalibrationError, CalibrationMode, ClosedLoopCalibration,
        ClosedLoopReconstructor, MixedMirrorMode, Reconstructor, algebra::CalibProps,
    },
    centroiding::CentroidsProcessing,
    crseo::{
        FromBuilder, Gmt, Imaging,
        gmt::{GmtM1, GmtM2},
    },
    sensors::builders::CameraBuilder,
};
use gmt_dos_systems_agws::builder::shack_hartmann::ShackHartmannBuilder;
use interface::filing::{Filing, FilingError};

use crate::agws::{
    TXY_RESIDUAL_SCALING, differential_reconstructor::{DSReconstructorError, DifferentialStackedReconstructor},
};

#[derive(Debug, thiserror::Error)]
pub enum Sh48CalibrationError {
    #[error("failed to write reconstructor to data repository")]
    SaveRecon(#[from] FilingError),
    #[error("failed to calibration SH48")]
    Calibrate(#[from] CalibrationError),
    #[error("failed to build differential & stacked reconstructor")]
    DSRecon(#[from] DSReconstructorError),
    #[error("Missing M1 bending modes reconstructor")]
    MissingM1Recon
}

pub trait Sh48Reconstructor {
    const TXY_RESIDUAL_SCALING: f64;
}
pub enum Merge {}
impl Sh48Reconstructor for Merge {
    const TXY_RESIDUAL_SCALING: f64 = 150.0;
}
pub enum Stack {}
impl Sh48Reconstructor for Stack {
    const TXY_RESIDUAL_SCALING: f64 = 1.0;
}
pub enum DiffStack {}
impl Sh48Reconstructor for DiffStack {
    const TXY_RESIDUAL_SCALING: f64 = 1.0;
}

pub trait M2RBMS {
    fn calibration_mode() -> CalibrationMode;
    fn file_name() -> String;
    fn to_string() -> String;
}
pub enum M2Txy {}
impl M2RBMS for M2Txy {
    fn calibration_mode() -> CalibrationMode {
        CalibrationMode::t_xy(1e-6)
    }
    fn file_name() -> String {
        "sh48_closed-loop_Txy_calib.pkl".to_string()
    }
    fn to_string() -> String {
        "txy".into()
    }
}
pub enum M2Txyz {}
impl M2RBMS for M2Txyz {
    fn calibration_mode() -> CalibrationMode {
        CalibrationMode::RBM([Some(1e-6), Some(1e-6), Some(1e-6), None, None, None])
    }
    fn file_name() -> String {
        "sh48_closed-loop_Txyz_calib.pkl".to_string()
    }
    fn to_string() -> String {
        "txyz".into()
    }
}

pub struct Sh48Calibration<R, T = M2Txy>
where
    R: Sh48Reconstructor,
    T: M2RBMS,
{
    m2_txy: ClosedLoopReconstructor,
    m1_bm: Option<Reconstructor>,
    m1_n_mode: usize,
    m1_modes: String,
    reconstructor_kind: PhantomData<R>,
    m2_rbms: PhantomData<T>,
}
type Result<T> = std::result::Result<T, Sh48CalibrationError>;

impl<R: Sh48Reconstructor, T: M2RBMS> Sh48Calibration<R, T> {
    pub fn new() -> Result<Self> {
        let file_name = <T as M2RBMS>::file_name();
        let recon: ClosedLoopReconstructor = if let Ok(recon) =
            ClosedLoopReconstructor::from_data_repo(&file_name)
        {
            recon
        } else {
            let sh48_omb: OpticalModelBuilder<CameraBuilder<1>> =
                (&ShackHartmannBuilder::<Reconstructor>::sh48().use_calibration_src()).into();
            let sh24_omb: OpticalModelBuilder<CameraBuilder<1>> =
                (&ShackHartmannBuilder::<Reconstructor>::sh24().use_calibration_src()).into();
            let recon = <CentroidsProcessing as ClosedLoopCalibration<GmtM2, Imaging>>::calibrate(
                &(&sh48_omb).into(),
                <T as M2RBMS>::calibration_mode(),
                &(&sh24_omb).into(),
                CalibrationMode::r_xy(1e-6),
            )?;
            recon.to_data_repo(file_name)?;
            recon
        };
        // println!("{recon}");
        Ok(Self {
            m2_txy: recon,
            m1_bm: None,
            m1_n_mode: 0,
            m1_modes: String::new(),
            reconstructor_kind: PhantomData,
            m2_rbms: PhantomData,
        })
    }
}
impl<R: Sh48Reconstructor, T: M2RBMS> Sh48Calibration<R, T> {
    pub fn m1_modes(self, m1_modes: &str, m1_n_mode: usize) -> Result<Self> {
        // calibration of M1 Sx bending modes with SH48
        let file_name = format!("sh48_{}-{}_calib.pkl", m1_n_mode, m1_modes);
        let m1_bm_recon: Reconstructor =
            if let Ok(recon) = Reconstructor::from_data_repo(&file_name) {
                recon
            } else {
                let sh48_omb: OpticalModelBuilder<CameraBuilder<1>> =
                    (&ShackHartmannBuilder::<Reconstructor>::sh48().use_calibration_src()).into();
                let gmtb = Gmt::builder().m1(m1_modes, m1_n_mode);
                let recon = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
                    &(&sh48_omb.gmt(gmtb.clone())).into(),
                    CalibrationMode::modes(m1_n_mode, 1e-6),
                )?;
                recon.to_data_repo(file_name)?;
                recon
            };
        // println!("{m1_bm_recon}");
        Ok(Self {
            m1_bm: Some(m1_bm_recon),
            m1_n_mode,
            m1_modes: m1_modes.to_string(),
            ..self
        })
    }
}
impl Sh48Calibration<Merge> {
    pub fn recon(self) -> Result<Reconstructor<MixedMirrorMode>> {
        Ok(if let Some(m1_bm_recon) = self.m1_bm {
            let file_name = format!(
                "sh48_merged_m2-txy_{}-{}_recon.pkl",
                self.m1_n_mode, self.m1_modes
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
                // println!("{recon}");
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
impl<T: M2RBMS> Sh48Calibration<Stack, T> {
    pub fn recon(self) -> Result<Reconstructor<MixedMirrorMode>> {
        Ok(if let Some(m1_bm_recon) = self.m1_bm {
            let file_name = format!(
                "sh48_stacked-{}_{}-{}_recon.pkl",
                <T as M2RBMS>::to_string(),
                self.m1_n_mode,
                self.m1_modes
            );
            if let Ok(recon) = Reconstructor::from_data_repo(&file_name) {
                recon
            } else {
                let mmode = MixedMirrorMode::from(vec![
                    <T as M2RBMS>::calibration_mode(),
                    CalibrationMode::modes(self.m1_n_mode, 1e-6),
                ]);
                let (c, d): (Vec<_>, Vec<_>) = self
                    .m2_txy
                    .calib()
                    .zip(m1_bm_recon.calib())
                    .enumerate()
                    .map(|(i, (c_txy, c_bms))| {
                        // M2 RBMs calibration matrix pseudo-inverse
                        let ic_txy = c_txy.pseudoinverse().unwrap();
                        // M1 bending modes calibration matrix pseudo-inverse
                        let mut ic_bms = c_bms.pseudoinverse().unwrap();
                        // Remove M2 RBMs command (as slopes) from
                        // M1 bending modes calibration matrix pseudo-inverse
                        ic_bms.transform(|mat| mat - mat * c_txy.mat_ref() * ic_txy.mat_ref());
                        // dbg!(&ic_bms);
                        // Concatenation of both calibration matrices
                        let mut d = faer::Mat::<f64>::zeros(
                            c_txy.mat_ref().nrows(),
                            c_txy.mat_ref().ncols() + c_bms.mat_ref().ncols(),
                        );
                        d.as_mut()
                            .subcols_mut(0, c_txy.mat_ref().ncols())
                            .copy_from(c_txy.mat_ref());
                        d.as_mut()
                            .subcols_mut(c_txy.mat_ref().ncols(), c_bms.mat_ref().ncols())
                            .copy_from(c_bms.mat_ref());
                        let c = Calib::<MixedMirrorMode>::builder()
                            .c(d.col_iter()
                                .flat_map(|c| c.iter().copied())
                                .collect::<Vec<_>>())
                            .sid(i as u8 + 1)
                            .mask(c_bms.mask_as_slice().to_vec())
                            .mode(mmode.clone())
                            .n_mode(self.m1_n_mode + 2)
                            .build();
                        // Concatenation of both pseudo-inverse matrices
                        let mut d = faer::Mat::<f64>::zeros(
                            c_txy.mat_ref().ncols() + c_bms.mat_ref().ncols(),
                            c_txy.mat_ref().nrows(),
                        );
                        d.as_mut()
                            .subrows_mut(0, c_txy.mat_ref().ncols())
                            .copy_from(ic_txy.mat_ref());
                        d.as_mut()
                            .subrows_mut(c_txy.mat_ref().ncols(), c_bms.mat_ref().ncols())
                            .copy_from(ic_bms.mat_ref());
                        (c, d)
                    })
                    .unzip();
                let mut recon = Reconstructor::<MixedMirrorMode>::new(c);
                recon.set_pinv(d).to_data_repo(&file_name)?;
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
impl<T: M2RBMS> Sh48Calibration<DiffStack, T> {
    pub fn recon(self) -> Result<DifferentialStackedReconstructor> {
        if let Some(m1_bm_recon) = self.m1_bm {
            let file_name = format!(
                "sh48_stacked-{}_{}-{}_recon.pkl",
                <T as M2RBMS>::to_string(),
                self.m1_n_mode,
                self.m1_modes
            );
            if let Ok(recon) = DifferentialStackedReconstructor::from_data_repo(&file_name) {
                Ok(recon)
            } else {
                let recon = DifferentialStackedReconstructor::new(3, self.m2_txy.clone(), m1_bm_recon)?;
                recon.to_data_repo(&file_name)?;
                Ok(recon)
            }
        } else {
                Err(Sh48CalibrationError::MissingM1Recon)
        }
    }
}

#[cfg(test)]
mod tests {
    //
    // comparison of the 3 reconstructor with
    // ```
    // cargo test -r --lib -- estimate --show-outputs
    // ```
    // The input opticasl state is set with the function `gmt_optical_state`
    //
    use std::{error::Error, fs::File, iter::Empty};

    use super::*;
    use gmt_dos_clients_io::{Estimate, gmt_m2::M2RigidBodyMotions};
    use gmt_dos_clients_optics_state::{MirrorState, OpticalState, OpticsState, SegmentState};
    use gmt_dos_systems_agws::{
        Agws,
        agws::{AgwsParts, sh24::Sh24TT},
        builder::shack_hartmann::ShackHartmannBuilder,
        kernels::KernelFrame,
    };
    use interface::{Data, Read, TryRead, TryUpdate, TryWrite, Update, Write};

    type K48 = crate::agws::Sh48Reconstructor<1>;
    type K24 = Sh24TT<1>;

    #[test]
    fn stack_recon() -> std::result::Result<(), Sh48CalibrationError> {
        Sh48Calibration::<Stack>::new()?
            .m1_modes(config::m1::segment::MODES, config::m1::segment::N_MODE)?
            .recon()?;
        Ok(())
    }

    fn print_rbms<'a>(
        data: impl Iterator<Item = &'a [f64]>,
        modes: Option<impl Iterator<Item = &'a [f64]>>,
    ) {
        if let Some(modes) = modes {
            println!("[M2 RBMS (x1e6)] M1 BMs RSS (x1e9)");
            data.map(|x| x.iter().map(|x| x * 1e6).collect::<Vec<_>>())
                .zip(
                    modes
                        // .inspect(|y| {
                        //     dbg!(y);
                        // })
                        .map(|x| x.iter().map(|x| x * 1e9).map(|x| x * x).sum::<f64>())
                        .map(|x| x.sqrt()),
                )
                .enumerate()
                .for_each(|(i, (x, y))| println!(" {}:{x:+5.1?} {y:+6.1}", i + 1));
        } else {
            println!("[M2 RBMS (x1e6)]");
            data.map(|x| x.iter().map(|x| x * 1e6).collect::<Vec<_>>())
                .enumerate()
                .for_each(|(i, x)| println!(" {}:{x:+5.1?}", i + 1));
        }
    }

    fn agws(
        reconstructor: Reconstructor<MixedMirrorMode>,
    ) -> std::result::Result<AgwsParts<1, 1, K48, K24>, Box<dyn Error>> {
        let sh24_recon: Reconstructor = serde_pickle::from_reader(
            File::open("calibrations/sh24/recon_sh24-to-rbm_pth.pkl")?,
            Default::default(),
        )?;
        let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);
        let agws_parts = Agws::<1, 1, K48, K24>::builder()
            .gmt(gmtb)
            .sh24(ShackHartmannBuilder::sh24().use_calibration_src())
            .sh24_calibration(sh24_recon)
            .sh48(ShackHartmannBuilder::sh48().use_calibration_src())
            .sh48_calibration(reconstructor)
            .parts()?;

        Ok(agws_parts)
    }

    fn estimation(
        AgwsParts {
            sh48,
            sh24,
            sh24_kernel,
            sh48_kernel,
        }: &mut AgwsParts<1, 1, K48, K24>,
    ) -> std::result::Result<Data<Estimate>, Box<dyn Error>> {
        let mut optical_state = gmt_optical_state();

        // SH24 M2 Rxy correction
        <_ as Read<OpticsState>>::read(sh24, Data::new(optical_state.clone()));
        sh24.update();
        let data = <_ as Write<KernelFrame<K24>>>::write(sh24).unwrap();

        <_ as TryRead<KernelFrame<K24>>>::try_read(sh24_kernel, data)?;
        sh24_kernel.try_update()?;
        let data = <_ as TryWrite<M2RigidBodyMotions>>::try_write(sh24_kernel)?.unwrap();

        optical_state.m2_as_mut().map(|m2| {
            m2.iter_mut()
                .zip(data.chunks(6))
                .for_each(|(segment, rbms)| {
                    segment.map(|segment| {
                        *segment = segment.clone() - SegmentState::rbms(rbms);
                    });
                })
        });

        println!("M2 RBMS w/ SH24 Rxy");
        let m2_rbms = optical_state
            .m2_as_mut()
            .and_then(|m2| m2.into_rbms())
            .unwrap();
        print_rbms(m2_rbms.chunks(6), Option::<Empty<&[f64]>>::None);

        // SH48 M2 Txy estimation
        <_ as Read<OpticsState>>::read(sh48, Data::new(optical_state));
        sh48.update();
        let data = <_ as Write<KernelFrame<K48>>>::write(sh48).unwrap();

        <_ as TryRead<KernelFrame<K48>>>::try_read(sh48_kernel, data)?;
        sh48_kernel.try_update()?;
        let data = <_ as TryWrite<Estimate>>::try_write(sh48_kernel)?.unwrap();

        Ok(data)
    }

    fn gmt_optical_state() -> OpticalState {
        let mut rbms = vec![0f64; 6];
        rbms[0] = 1e-6; // Tx
        rbms[1] = 1e-6; // Ty
        rbms[2] = 1e-6; // Tz
        // OpticalState::m2(MirrorState::from(SegmentState::rbms(rbms)))
        OpticalState::new(
            MirrorState::from(SegmentState::rbms(rbms)),
            MirrorState::rbms(),
        )
        // OpticalState::new(
        //     MirrorState::from(
        //         SegmentState::modes(vec![0f64; config::m1::segment::N_MODE])
        //             .set_mode(config::m1::segment::N_MODE-1, 1e-7),
        //     ),
        //     MirrorState::rbms(),
        // )
    }

    #[test]
    fn m2_estimate() -> std::result::Result<(), Box<dyn Error>> {
        let recon = Sh48Calibration::<Merge, M2Txy>::new()?.recon()?;
        let mut agws_parts = agws(recon)?;
        let data = estimation(&mut agws_parts)?;
        println!("M2 RBMS Txy estimation with SH48");
        print_rbms(data.chunks(6), Option::<Empty<&[f64]>>::None);
        Ok(())
    }

    #[test]
    fn merge_estimate() -> std::result::Result<(), Box<dyn Error>> {
        let recon = Sh48Calibration::<Merge, M2Txy>::new()?
            .m1_modes(config::m1::segment::MODES, config::m1::segment::N_MODE)?
            .recon()?;
        let mut agws_parts = agws(recon)?;
        let data = estimation(&mut agws_parts)?;
        println!("M2 RBMS Txy(z) estimation with SH48");
        let rbms: Vec<f64> = data
            .chunks(6 + config::m1::segment::N_MODE)
            .flat_map(|x| {
                x.iter()
                    .take(6)
                    .map(|x| x * <Merge as Sh48Reconstructor>::TXY_RESIDUAL_SCALING)
            })
            .collect();
        let modes = data
            .chunks(6 + config::m1::segment::N_MODE)
            .map(|x| &x[6..]);
        print_rbms(rbms.chunks(6), Some(modes));
        Ok(())
    }

    #[test]
    fn stack_estimate() -> std::result::Result<(), Box<dyn Error>> {
        let recon = Sh48Calibration::<Stack, M2Txyz>::new()?
            .m1_modes(config::m1::segment::MODES, config::m1::segment::N_MODE)?
            .recon()?;
        let mut agws_parts = agws(recon)?;
        let data = estimation(&mut agws_parts)?;
        println!("M2 RBMS Txy(z) estimation with SH48");
        let rbms = data
            .chunks(6 + config::m1::segment::N_MODE)
            .map(|x| &x[..6]);
        let modes = data
            .chunks(6 + config::m1::segment::N_MODE)
            .map(|x| &x[6..]);
        print_rbms(rbms, Some(modes));
        Ok(())
    }
}
