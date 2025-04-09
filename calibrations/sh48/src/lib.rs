use std::{error::Error, fmt::Display, io};

use gmt_dos_clients_crseo::calibration::CalibrationError;

pub mod closed_loop;
pub mod open_loop;

#[derive(Debug)]
pub enum SH48CalibrationError {
    Calibration(CalibrationError),
    Serde(serde_pickle::Error),
    IO(io::Error),
}

impl Display for SH48CalibrationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "SH48 calibration error due to")?;
        match self {
            SH48CalibrationError::Calibration(calibration_error) => calibration_error.fmt(f),
            SH48CalibrationError::Serde(error) => error.fmt(f),
            SH48CalibrationError::IO(error) => error.fmt(f),
        }
    }
}
impl Error for SH48CalibrationError {}
impl From<CalibrationError> for SH48CalibrationError {
    fn from(value: CalibrationError) -> Self {
        Self::Calibration(value)
    }
}
impl From<serde_pickle::Error> for SH48CalibrationError {
    fn from(value: serde_pickle::Error) -> Self {
        Self::Serde(value)
    }
}
impl From<io::Error> for SH48CalibrationError {
    fn from(value: io::Error) -> Self {
        Self::IO(value)
    }
}
