use std::sync::Arc;

use faer::MatRef;
use gmt_dos_clients_crseo::calibration::{
    ClosedLoopReconstructor, Reconstructor,
    algebra::{CalibError, CalibProps},
};
use gmt_dos_clients_io::{Estimate, optics::SensorData};
use interface::{Data, Read, UniqueIdentifier, Update, Write, filing::Codec};
use serde::{Deserialize, Serialize};

#[derive(Debug, thiserror::Error)]
pub enum DSReconstructorError {
    #[error("failed to build DifferentialStackedReconstructor")]
    New(#[from] CalibError),
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct DifferentialStackedReconstructor {
    m2: ClosedLoopReconstructor,
    m1: Reconstructor,
    n_guide_star: usize,
    diff_recon: Reconstructor,
    data: Arc<Vec<f64>>,
    estimate: Arc<Vec<f64>>,
}

impl DifferentialStackedReconstructor {
    pub fn new(
        n_guide_star: usize,
        m2: ClosedLoopReconstructor,
        m1: Reconstructor,
    ) -> Result<Self, DSReconstructorError> {
        let mut diff_recon = m2.guide_stars_differentiation(n_guide_star)?;
        diff_recon.pseudoinverse();
        println!("{diff_recon}");
        Ok(Self {
            m2,
            m1,
            n_guide_star,
            diff_recon,
            ..Default::default()
        })
    }
}

impl Update for DifferentialStackedReconstructor {
    fn update(&mut self) {
        // differential splopes
        let n = self.data.len() / self.n_guide_star;
        let mut w = self.data.chunks(n).cycle().peekable();
        let diff_slopes: Vec<_> = (0..3)
            .flat_map(|_| {
                w.next()
                    .unwrap()
                    .iter()
                    .zip(w.peek().unwrap().iter())
                    .map(|(x, y)| x - y)
                    .collect::<Vec<_>>()
            })
            .collect();
        // M2 RBMs
        <_ as Read<SensorData>>::read(&mut self.diff_recon, diff_slopes.into());
        self.diff_recon.update();
        let rbms = <_ as Write<Estimate>>::write(&mut self.diff_recon).unwrap();
        // M1 BMs
        let mut estimates = Vec::<f64>::new();
        if self.m1.pinv_as_ref().is_none() {
            self.m1.pseudoinverse();
        }
        for ((calib_m2, rbms), imat) in self.m2.calib().zip(rbms.chunks(6)).zip(self.m1.pinv_iter())
        {
            // M2 slopes
            let s_t: Vec<_> = (calib_m2.mat_ref()
                * MatRef::from_column_major_slice(&rbms[..2], 2, 1))
            .col(0)
            .iter()
            .copied()
            .collect::<Vec<f64>>();
            // slopes
            let s_b: Vec<_> = self
                .data
                .iter()
                .zip(calib_m2.mask_as_slice())
                .filter_map(|(d, m)| m.then(|| *d))
                .collect();
            // slopes - M2 slopes
            let s: Vec<_> = s_b
                .into_iter()
                .zip(s_t.into_iter())
                .map(|(x, y)| x - y)
                .collect();
            // M1 BMs
            let m = imat * MatRef::from_column_major_slice(&s, s.len(), 1);
            estimates.extend(rbms);
            estimates.extend(m.col(0).iter());
        }
        self.estimate = estimates.into();
    }
}

impl<U> Read<U> for DifferentialStackedReconstructor
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
{
    fn read(&mut self, data: Data<U>) {
        self.data = data.into_arc();
    }
}

impl<U> Write<U> for DifferentialStackedReconstructor
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
{
    fn write(&mut self) -> Option<Data<U>> {
        Some(self.estimate.clone().into())
    }
}

impl Codec for DifferentialStackedReconstructor {}
