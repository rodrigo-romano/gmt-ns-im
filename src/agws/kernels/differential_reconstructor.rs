use gmt_dos_clients::integrator::Integrator;
use gmt_dos_clients_crseo::{
    DeviceInitialize, OpticalModelBuilder, centroiding::CentroidsProcessing, crseo::FromBuilder, sensors::Camera
};
use gmt_dos_clients_io::{
    Estimate,
    optics::{Dev, Frame, SensorData},
};
use gmt_dos_systems_agws::kernels::{KernelError, KernelSpecs};

use crate::agws::differential_reconstructor::DifferentialStackedReconstructor;

pub struct Sh48DiffReconstructor<const I: usize>;

impl<const I: usize> KernelSpecs for Sh48DiffReconstructor<I> {
    type Sensor = Camera<I>;

    type Processor = CentroidsProcessing;

    type Estimator = DifferentialStackedReconstructor;
    // type Estimator = Reconstructor<CalibrationMode, ClosedLoopCalib>;

    type Controller = Integrator<Estimate>;

    type Input = Frame<Dev>;

    type Data = SensorData;

    type Output = Estimate;

    fn processor(
        model: &OpticalModelBuilder<<Self::Sensor as FromBuilder>::ComponentBuilder>,
    ) -> std::result::Result<Self::Processor, KernelError> {
        let mut centroids = CentroidsProcessing::try_from(model)?;
        model.initialize(&mut centroids);
        Ok(centroids)
    }
}
