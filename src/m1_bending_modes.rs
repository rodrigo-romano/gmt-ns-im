use std::{fs::File, path::Path, sync::Arc};

use gmt_dos_clients_io::gmt_m1::{M1ModeShapes, assembly::M1ModeCoefficients};
use gmt_dos_systems_m1::SingularModes;
use interface::{Data, Read, Update, Write};

use crate::config;

#[derive(Debug, Default, Clone)]
pub struct M1BendingModes {
    modes: Vec<SingularModes>,
    surfaces: Arc<Vec<f64>>,
    coefs: Arc<Vec<f64>>,
}

impl M1BendingModes {
    pub fn new(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let modes: Vec<SingularModes> =
            serde_pickle::from_reader(&mut File::open(path.as_ref())?, Default::default())?;
        Ok(Self {
            modes,
            ..Default::default()
        })
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
