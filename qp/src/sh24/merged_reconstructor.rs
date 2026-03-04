use gmt_dos_clients::integrator::Integrator;
use gmt_dos_clients_crseo::{
    DeviceInitialize, OpticalModelBuilder, calibration::{Calib, MixedMirrorMode, Reconstructor}, centroiding::CentroidsProcessing, crseo::FromBuilder, sensors::Camera
};
use gmt_dos_clients_io::{
    Estimate,
    optics::{Dev, Frame, SensorData},
};
use gmt_dos_systems_agws::kernels::{KernelError, KernelSpecs};

pub struct Sh48MergerReconstructor<const I: usize>;

impl<const I: usize> KernelSpecs for Sh48MergerReconstructor<I> {
    type Sensor = Camera<I>;

    type Processor = CentroidsProcessing;

    type Estimator = Reconstructor<MixedMirrorMode, Calib<MixedMirrorMode>>;
    // type Estimator = Reconstructor<CalibrationMode, ClosedLoopCalib>;

    type Integrator = Integrator<Estimate>;

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
