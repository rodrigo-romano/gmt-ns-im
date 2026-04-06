use std::{fs::File, iter, path::Path, sync::Arc};

use gmt_dos_clients_io::gmt_m1::{M1ModeShapes, assembly::M1ModeCoefficients};
// use gmt_dos_clients_optics_state::MirrorState;
use gmt_dos_systems_m1::SingularModes;
use interface::{Data, Read, Update, Write};

/// Projection of M1 segment figures onto M1 segment bending modes
#[derive(Debug, Clone)]
pub struct M1BendingModes {
    // bending modes data structure
    modes: SingularModes,
    // segment figures
    surfaces: Arc<Vec<f64>>,
    // bending modes coefficients
    coefs: Arc<Vec<f64>>,
    // M1 optical state
    // state: Arc<MirrorState>,
}

impl M1BendingModes {
    pub fn new(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let modes: SingularModes =
            serde_pickle::from_reader(&mut File::open(path.as_ref())?, Default::default())?;
        Ok(Self {
            modes,
            surfaces: Default::default(),
            coefs: Default::default(),
        })
    }
}
impl From<M1BendingModes> for Vec<nalgebra::DMatrix<f64>> {
    fn from(m1_bms: M1BendingModes) -> Self {
        m1_bms
            .modes
            .into_iter()
            .map(|sms| {
                let (ns, na) = sms.shape();
                nalgebra::DMatrix::from_iterator(
                    ns,
                    config::m1::segment::N_RAW_MODE,
                    sms.raw_modes_iter().copied().chain(iter::repeat_n(
                        0f64,
                        (config::m1::segment::N_RAW_MODE - na) * ns,
                    )),
                )
                .transpose()
            })
            .collect()
    }
}
impl Update for M1BendingModes {
    fn update(&mut self) {
        let mut ns_acc = 0;
        self.coefs = Arc::new(
            self.modes
                .iter()
                .flat_map(|mode| {
                    let mat = mode.mat_ref();
                    let (ns, na) = mat.shape();
                    // let mat =
                    //     faer::mat::Mat::from_column_major_slice::<f64>(&mode.raw_modes, ns, na);
                    let deltas = &self.surfaces[ns_acc..ns_acc + ns];
                    ns_acc += ns;
                    let coefs =
                        mat.transpose() * faer::mat::MatRef::from_column_major_slice(deltas, ns, 1);
                    let mut coefs = coefs.col_as_slice(0).to_vec();
                    coefs.extend(vec![0f64; config::m1::segment::N_RAW_MODE - na]);
                    coefs
                })
                .collect(),
        );
    }
}

impl Read<M1ModeShapes> for M1BendingModes {
    fn read(&mut self, data: Data<M1ModeShapes>) {
        self.surfaces = data.into_arc();
    }
}
impl Write<M1ModeCoefficients> for M1BendingModes {
    fn write(&mut self) -> Option<Data<M1ModeCoefficients>> {
        Some(self.coefs.clone().into())
    }
}

// impl Read<M1State> for M1BendingModes {
//     fn read(&mut self, data: Data<M1State>) {
//         self.state = data.into_arc();
//         self.surfaces = Arc::new(self.state.into_modes().unwrap_or_default());
//     }
// }
// impl Write<M1State> for M1BendingModes {
//     fn write(&mut self) -> Option<Data<M1State>> {
//         let state = MirrorState {
//             rbms: Arc::new(self.state.into_rbms().unwrap_or_default()),
//             modes: Some(self.coefs.clone()),
//         };
//         Some(Data::new(state))
//     }
// }
