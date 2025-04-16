use std::{
    any::type_name, error::Error, fmt::Display, fs::File, io, marker::PhantomData, path::Path,
    sync::Arc,
};

use faer::MatRef;
use gmt_dos_clients_crseo::calibration::{
    Calib, CalibrationMode, ClosedLoopCalib, Modality, Reconstructor, SegmentMode,
    algebra::CalibProps,
};
use gmt_dos_clients_io::{gmt_m2::M2RigidBodyMotions, optics::M1Modes};
use interface::{Data, OperatorLeftRight, Read, UID, UniqueIdentifier, Update, Write};

#[derive(Debug)]
pub enum MergeError {
    Open(io::Error),
    Pickle(serde_pickle::Error),
}

impl Display for MergeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MergeError::Open(error) => error.fmt(f),
            MergeError::Pickle(error) => error.fmt(f),
        }
    }
}
impl Error for MergeError {}
impl From<io::Error> for MergeError {
    fn from(value: io::Error) -> Self {
        Self::Open(value)
    }
}
impl From<serde_pickle::Error> for MergeError {
    fn from(value: serde_pickle::Error) -> Self {
        Self::Pickle(value)
    }
}

pub struct MergeReconstructor<M: Modality + Display, A, B, C = ()> {
    recon: Reconstructor<M, Calib<M>>,
    data: Arc<Vec<f64>>,
    estimates: (Arc<Vec<f64>>, Arc<Vec<f64>>, Arc<Vec<f64>>),
    // estimate_sizes: Vec<(usize, usize, usize)>,
    a: PhantomData<A>,

    b: PhantomData<B>,
    c: PhantomData<C>,
}
impl<M: Modality + Display + Default, A, B, C> Display for MergeReconstructor<M, A, B, C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "Merge Reconstructor [{},{},{}]:",
            type_name::<A>(),
            type_name::<B>(),
            type_name::<C>()
        )?;
        self.recon.fmt(f)?;
        Ok(())
    }
}

impl MergeReconstructor<SegmentMode, M2RigidBodyMotions, M1Modes> {
    pub fn new(
        a: impl AsRef<Path>,
        b: impl AsRef<Path>,
        svd_truncation: Option<Vec<usize>>,
    ) -> Result<Self, MergeError> {
        let file = File::open(a.as_ref())?;
        let mut recon_a: Reconstructor<CalibrationMode, ClosedLoopCalib> =
            serde_pickle::from_reader(&file, Default::default())?;
        let file = File::open(b.as_ref())?;
        let mut recon_b: Reconstructor<CalibrationMode, ClosedLoopCalib> =
            serde_pickle::from_reader(&file, Default::default())?;
        let ((calibs, sizes), nrms): ((Vec<_>, Vec<_>), Vec<_>) = recon_a
            .calib_slice_mut()
            .iter_mut()
            .zip(recon_b.calib_slice_mut())
            .map(|(ca, cb)| {
                assert_eq!(ca.n_rows(), cb.n_rows());

                let ca_nrm = ca.normalize();
                let cb_nrm = cb.normalize();

                let na = ca.n_cols();
                let nb = cb.n_cols();

                let mut mat = ca.as_slice().to_vec();
                mat.extend(cb.as_slice());
                let calib = Calib::builder()
                    .c(mat)
                    .n_cols(nb + na)
                    .mask(ca.mask_as_slice().to_vec())
                    .mode(SegmentMode::new(ca.mode(), cb.mode()))
                    .build();
                ((calib, (na, nb, 0)), (ca_nrm, cb_nrm))
            })
            .unzip();
        let mut recon = Reconstructor::new(calibs);
        if let Some(n) = svd_truncation {
            recon.truncated_pseudoinverse(n)
        } else {
            recon.pseudoinverse()
        };
        recon
            .pinv()
            .zip(&sizes)
            .zip(&nrms)
            .for_each(|((p, &(na, nb, _nc)), &(ca_nrm, cb_nrm))| {
                let mut l = faer::mat::Mat::<f64>::identity(nb + na, nb + na);
                l.diagonal_mut()
                    .column_vector_mut()
                    .iter_mut()
                    .take(na)
                    .for_each(|x| *x /= ca_nrm);
                l.diagonal_mut()
                    .column_vector_mut()
                    .iter_mut()
                    .skip(na)
                    .for_each(|x| *x /= cb_nrm);
                p.transform(|x| &l * x);
            });
        Ok(Self {
            recon,
            a: PhantomData,
            b: PhantomData,
            c: PhantomData,
            data: Default::default(),
            estimates: Default::default(),
            // estimate_sizes: sizes,
        })
    }
}
impl MergeReconstructor<CalibrationMode, M1Modes, ()> {
    pub fn single(a: impl AsRef<Path>) -> Result<Self, MergeError> {
        let file = File::open(a.as_ref())?;
        let mut recon_a: Reconstructor<CalibrationMode, ClosedLoopCalib> =
            serde_pickle::from_reader(&file, Default::default())?;
        let (calibs, _sizes): (Vec<_>, Vec<_>) = recon_a
            .calib_slice_mut()
            .iter_mut()
            .map(|ca| {
                // let ca_nrm = ca.normalize();

                let na = ca.n_cols();
                // let mut l = faer::mat::Mat::<f64>::identity(na, na);
                // l.diagonal_mut()
                //     .column_vector_mut()
                //     .iter_mut()
                //     .for_each(|x| *x /= ca_nrm);

                let mat = ca.as_slice().to_vec();
                let calib = Calib::builder()
                    .c(mat)
                    .n_cols(na)
                    .mask(ca.mask_as_slice().to_vec())
                    .mode(CalibrationMode::None)
                    .build();
                // calib.transform(|x| &l * x);
                (calib, (na, 0, 0))
            })
            .unzip();
        let mut recon = Reconstructor::new(calibs);
        let nrms = recon.normalize();
        recon.pseudoinverse();
        recon.pinv().zip(&nrms).for_each(|(p, n)| {
            p.transform(|x| x / *n);
        });
        Ok(Self {
            recon,
            a: PhantomData,
            b: PhantomData,
            c: PhantomData,
            data: Default::default(),
            estimates: Default::default(),
            // estimate_sizes: sizes,
        })
    }
}

impl<A, B> Update for MergeReconstructor<SegmentMode, A, B>
where
    A: UniqueIdentifier,
    B: UniqueIdentifier,
{
    fn update(&mut self) {
        // dbg!(self.data.len());
        // let mut u = 0;
        // let (ya, yb): (Vec<_>, Vec<_>) = self
        //     .recon
        //     .iter()
        //     .zip(&self.estimate_sizes)
        //     .map(|(r, (na, _nb, _nc))| {
        //         let m = r.mat_ref();
        //         let n = m.ncols();
        //         let s = &self.data[u..u + n];
        //         u += n;
        //         dbg!(u);
        //         let y = m * MatRef::from_column_major_slice(s, n, 1);
        //         let (ya, yb) = y.col_as_slice(0).split_at(*na);
        //         (ya.to_vec(), yb.to_vec())
        //     })
        //     .unzip();
        // dbg!(yb.iter().map(|x| { x.len() }).collect::<Vec<_>>());
        let (ya, yb): (Vec<_>, Vec<_>) = self
            .recon
            .calib_pinv()
            .map(|(c, ic)| {
                let rhs = c.mask(&self.data);
                let y = ic * MatRef::from_column_major_slice(rhs.as_slice(), rhs.len(), 1);
                c.mode().fill_split(y.col_as_slice(0).to_vec().into_iter())
            })
            // .zip(&self.estimate_sizes)
            // .map(|(y, (na, nb, nc))| {
            //     dbg!(y.len());
            //     let (ya, yb) = y.split_at(*na);
            //     (ya.to_vec(), yb.to_vec())
            // })
            .unzip();
        self.estimates = (
            Arc::new(ya.into_iter().flatten().collect()),
            Arc::new(yb.into_iter().flatten().collect()),
            Default::default(),
        )
    }
}

impl<U, A, B> Read<U> for MergeReconstructor<SegmentMode, A, B>
where
    U: UniqueIdentifier<DataType = Vec<f64>>,
    A: UniqueIdentifier,
    B: UniqueIdentifier,
{
    fn read(&mut self, data: Data<U>) {
        self.data = data.into_arc();
    }
}

#[derive(UID)]
pub enum SplitEstimate<const I: usize> {}

impl<const I: usize> OperatorLeftRight for SplitEstimate<I> {
    const LEFT: bool = true;
}

impl<A, B, const I: usize> Write<SplitEstimate<I>> for MergeReconstructor<SegmentMode, A, B>
where
    A: UniqueIdentifier,
    B: UniqueIdentifier,
{
    fn write(&mut self) -> Option<Data<SplitEstimate<I>>> {
        Some(
            match I {
                0 => self.estimates.0.clone(),
                1 => self.estimates.1.clone(),
                _ => panic!("Found SplitEstimate #{I}, expected 0 or 1"),
            }
            .into(),
        )
    }
}
// impl<B> Write<M2RigidBodyMotions> for MergeReconstructor<M2RigidBodyMotions, B>
// where
//     B: UniqueIdentifier,
// {
//     fn write(&mut self) -> Option<Data<M2RigidBodyMotions>> {
//         Some(self.estimates.0.clone().into())
//     }
// }
// impl<A> Write<M1Modes> for MergeReconstructor<A, M1Modes>
// where
//     A: UniqueIdentifier,
// {
//     fn write(&mut self) -> Option<Data<M1Modes>> {
//         Some(self.estimates.1.clone().into())
//     }
// }

#[cfg(test)]
mod tests {
    pub use super::*;

    #[test]
    fn merge() -> Result<(), Box<dyn Error>> {
        let sh48_merge_recon = MergeReconstructor::new(
            "calibrations/sh48/closed_loop_recon_sh48-to-m2-rbm.pkl",
            "calibrations/sh48/closed_loop_recon_sh48-to-m1-bm.pkl",
            None,
        )?;
        println!("{sh48_merge_recon}");
        Ok(())
    }
}
