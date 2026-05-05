use std::{
    env,
    error::Error,
    fs::{self, File},
    path::Path,
};

use faer::MatRef;
use gmt_dos_clients::gif::{self, FrameBuilder};
use gmt_dos_clients_crseo::{
    DeviceInitialize, OpticalModel, OpticalModelBuilder,
    calibration::{
        ClosedLoopReconstructor, Reconstructor, algebra::CalibProps,
    },
    centroiding::CentroidsProcessing,
    crseo::{FromBuilder, Gmt},
    sensors::{Camera, builders::WaveSensorBuilder},
};
use gmt_dos_clients_io::{
    Estimate,
    gmt_m1::M1ModeShapes,
    gmt_m2::M2RigidBodyMotions,
    optics::{Dev, Frame, SensorData, Wavefront},
};
use gmt_dos_clients_optics_state::{MirrorState, OpticalState, OpticsState, SegmentState};
use gmt_dos_systems_agws::{
    Agws,
    agws::{AgwsParts, sh24::Sh24TT, sh48::Sh48},
    builder::shack_hartmann::{AgwsGuideStar, ShackHartmannBuilder},
    kernels::{KernelError, KernelFrame, KernelSpecs},
};
use interface::{Data, Read, TryRead, TryUpdate, TryWrite, Update, Write};

const N: usize = 865;

fn m2_rbms() -> Vec<f64> {
    let mut m2_rbms = vec![vec![0f64; 6]; 7];
    m2_rbms[0][0] = 1e-6;
    m2_rbms.into_iter().flatten().collect()
}
fn m1_modes() -> Vec<f64> {
    let m1_modes = vec![vec![0f64; config::m1::segment::N_MODE]; 7];
    // m1_modes[0][26] = 1e-6;
    // m1_modes[0][0] = 1e-6;
    m1_modes.into_iter().flatten().collect()
}

#[test]
fn wavesensor() -> Result<(), Box<dyn Error>> {
    let data_repo = Path::new(&env::var("DATA_REPO")?)
        .join("tests")
        .join("sh48-wavefronts");
    fs::create_dir_all(&data_repo)?;
    unsafe {
        env::set_var("DATA_REPO", data_repo);
    }

    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);

    let omb = OpticalModel::<Camera>::builder()
        .gmt(gmtb)
        .source(AgwsGuideStar::sh48().pupil_sampling(N));
    let mut om = OpticalModelBuilder::<WaveSensorBuilder>::from(&omb).build()?;
    println!("{om}");

    let mut agws_wavefronts: gif::Frame<f64> = gif::Frame::new("agws_wavefronts.png", N);
    let mut agws_diff_wavefronts: gif::Frame<f64> = gif::Frame::new("agws_diff_wavefronts.png", N);

    <_ as Read<M1ModeShapes>>::read(&mut om, m1_modes().into());
    om.update();
    let wavefronts = <_ as Write<Wavefront>>::write(&mut om).unwrap();
    let min = 1e9
        * wavefronts
            .iter()
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap();
    let max = 1e9
        * wavefronts
            .iter()
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap();
    println!("Wavefronts min/max: {:.0?}nm", (min, max));
    let mut w = wavefronts.chunks(N * N).cycle().peekable();
    let diff_wavefronts: Vec<_> = (0..3)
        .flat_map(|_| {
            w.next()
                .unwrap()
                .iter()
                .zip(w.peek().unwrap().iter())
                .map(|(x, y)| {
                    if x.abs() > 0f64 && y.abs() > 0f64 {
                        x - y
                    } else {
                        0f64
                    }
                })
                .collect::<Vec<_>>()
        })
        .collect();
    let min = 1e9
        * diff_wavefronts
            .iter()
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap();
    let max = 1e9
        * diff_wavefronts
            .iter()
            .max_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap();
    println!("Diff. Wavefronts min/max: {:.0?}nm", (min, max));
    <_ as Read<Wavefront>>::read(&mut agws_wavefronts, wavefronts.into());
    agws_wavefronts.update();
    <_ as Read<Wavefront>>::read(&mut agws_diff_wavefronts, diff_wavefronts.into());
    agws_diff_wavefronts.update();

    Ok(())
}

type K24 = Sh24TT<1>;

pub struct Sh48M1to2<const I: usize>;
impl<const I: usize> KernelSpecs for Sh48M1to2<I> {
    type Sensor = Camera<I>;

    type Processor = CentroidsProcessing;

    type Estimator = ClosedLoopReconstructor;

    type Controller = gmt_dos_clients::integrator::Integrator<Estimate>;

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

#[test]
fn slopesensor() -> Result<(), Box<dyn Error>> {
    type K48 = Sh48<1>;
    type K24 = Sh24TT<1>;
    type AgwsSh48Frame = KernelFrame<K48>;
    let data_repo = Path::new(&env::var("DATA_REPO")?)
        .join("tests")
        .join("sh48-wavefronts");
    fs::create_dir_all(&data_repo)?;
    unsafe {
        env::set_var("DATA_REPO", data_repo);
    }

    fn sh24_estimation(
        optical_state: &mut OpticalState,
        AgwsParts {
            sh24, sh24_kernel, ..
        }: &mut AgwsParts<1, 1, K48, K24>,
    ) -> std::result::Result<(), Box<dyn Error>> {
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
        Ok(())
    }

    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);

    // let recon: Reconstructor = serde_pickle::from_reader(
    //     File::open(
    //         "/home/ubuntu/projects/gmt-ns-im/web_server/static/main/sh48_closed-loop_Txy_calib.pkl",
    //     )?,
    //     Default::default(),
    // )?;
    let sh24_recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/sh24/recon_sh24-to-rbm_pth.pkl")?,
        Default::default(),
    )?;

    let mut parts = Agws::<1, 1, K48, K24>::builder()
        .gmt(gmtb.clone())
        .sh24(ShackHartmannBuilder::sh24().use_calibration_src())
        .sh24_calibration(sh24_recon)
        .sh48(ShackHartmannBuilder::sh48().use_calibration_src())
        // .sh48_calibration(recon)
        .parts()?;

    let mut agws_slopes: gif::Frame<f64> =
        gif::Frame::new("agws_slopes.png", 48).image_size(256)?;
    let mut agws_diff_slopes: gif::Frame<f64> =
        gif::Frame::new("agws_diff_slopes.png", 48).image_size(256)?;

    let m1 = MirrorState::default()
        .zeros_modes(config::m1::segment::N_MODE)
        .set_segment_state(
            1,
            SegmentState::modes({
                let mut modes = vec![0f64; config::m1::segment::N_MODE];
                modes[0] = 1e-6;
                modes
            }),
        );
    let m2 = MirrorState::rbms().set_segment_state(
        1,
        SegmentState::rbms({
            let mut rbms = vec![0f64; 6];
            rbms[3] = 1e-6;
            rbms
        }),
    );
    let mut optical_state = OpticalState::new(m1, m2);

    sh24_estimation(&mut optical_state, &mut parts)?;
    dbg!(&optical_state);

    let AgwsParts {
        sh48,
        sh48_kernel,
        sh24,
        sh24_kernel,
    } = &mut parts;
    println!("{sh48}");
    let v = sh48_kernel.processor().get_valid_lenslets().to_vec();
    let n_v = v.iter().map(|v| *v as usize).sum::<usize>();
    println!("valid lenset #: {n_v}");

    // <_ as Read<M2RigidBodyMotions>>::read(&mut sh48, m2_rbms().into());
    // <_ as Read<M1ModeShapes>>::read(&mut sh48, m1_modes().into());
    <_ as Read<OpticsState>>::read(sh48, Data::new(optical_state.clone()));
    sh48.update();
    let sh48_frame = <_ as Write<AgwsSh48Frame>>::write(sh48).unwrap();
    <_ as TryRead<AgwsSh48Frame>>::try_read(sh48_kernel, sh48_frame)?;
    sh48_kernel.try_update()?;
    let mut _data = <_ as TryWrite<SensorData>>::try_write(sh48_kernel)?.unwrap();

    let mut data = _data.as_slice().to_vec();
    data.chunks_mut(48 * 48 * 2)
        .zip(v.chunks(48 * 48))
        .for_each(|(data, v)| {
            data.chunks_mut(48 * 48).for_each(|data| {
                data.iter_mut().zip(v).for_each(|(data, v)| {
                    if *v == 0 {
                        *data = 0f64
                    }
                })
            })
        });

    let min = data
        .iter()
        .min_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap();
    let max = data
        .iter()
        .max_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap();
    println!("Slopes min/max: ({:.3e},{:.3e})", min, max);
    let mut w = data.chunks(48 * 48 * 2).cycle().peekable();
    let diff_slopes: Vec<_> = (0..3)
        .flat_map(|_| {
            w.next()
                .unwrap()
                .iter()
                .zip(w.peek().unwrap().iter())
                .map(|(x, y)| {
                    if x.abs() > 0f64 && y.abs() > 0f64 {
                        x - y
                    } else {
                        0f64
                    }
                })
                .collect::<Vec<_>>()
        })
        .collect();
    let min = diff_slopes
        .iter()
        .min_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap();
    let max = diff_slopes
        .iter()
        .max_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap();
    println!("Diff. slopes min/max: ({:.3e},{:.3e})", min, max);
    <_ as Read<SensorData>>::read(&mut agws_slopes, data.clone().into());
    agws_slopes.update();
    <_ as Read<SensorData>>::read(&mut agws_diff_slopes, diff_slopes.clone().into());
    agws_diff_slopes.update();

    let mut recon: ClosedLoopReconstructor = serde_pickle::from_reader(
        File::open(
            "/home/ubuntu/projects/gmt-ns-im/web_server/static/main/sh48_closed-loop_Txy_calib.pkl",
        )?,
        Default::default(),
    )?;
    recon.pseudoinverse();
    println!("{recon}");

    <_ as Read<SensorData>>::read(&mut recon, data.clone().into());
    recon.update();
    let estimate = <_ as Write<Estimate>>::write(&mut recon).unwrap();
    dbg!(&estimate);

    let calib = recon.calib().nth(0).unwrap();
    // the centroids are given as [[cx,cy]_GS1,[cx,cy]_GS2,...]
    // the centroids mask has the same size than the full centroids
    // i.e. (# lenslet)^2 * (# guide star) * 2
    let mask = calib.mask_as_slice();
    dbg!(mask.len());
    // getting the # of non zeros elements in the mask
    let nz = mask
        .iter()
        .filter(|&&x| x)
        .enumerate()
        .map(|(i, _)| i)
        .last()
        .unwrap()
        + 1;
    println!("non zeros: {nz}");
    // getting the # of non zeros elements in the mask per guide star
    let nzs: Vec<_> = mask
        .chunks(mask.len() / 3)
        .map(|mask| {
            mask.iter()
                .filter(|&&x| x)
                .enumerate()
                .map(|(i, _)| i)
                .last()
                .unwrap()
                + 1
        })
        .collect();
    println!("non zeros: {nzs:?}={}", nzs.iter().sum::<usize>());

    // let diff_calib = calib.guide_stars_differentiation(3)?;
    // let mut diff_recon = Reconstructor::from(diff_calib);
    let mut diff_recon = recon.guide_stars_differentiation(3)?;
    diff_recon.pseudoinverse();
    println!("{diff_recon}");

    serde_pickle::to_writer(
        &mut File::create("differential_slopes.pkl")?,
        &(&diff_recon, &diff_slopes),
        Default::default(),
    )?;

    <_ as Read<SensorData>>::read(&mut diff_recon, diff_slopes.into());
    diff_recon.update();
    let estimate = <_ as Write<Estimate>>::write(&mut diff_recon).unwrap();
    dbg!(&estimate);

    let calib_m2_txy = recon.calib().nth(0).unwrap().mat_ref();
    // dbg!(calib_m2_txy.shape());
    let s_t: Vec<_> = (calib_m2_txy * MatRef::from_column_major_slice(&estimate[..2], 2, 1))
        .col(0)
        .iter()
        .copied()
        .collect::<Vec<f64>>();
    // dbg!(s_t.len());
    // dbg!(data.len());
    let s_b: Vec<_> = data
        .iter()
        .zip(mask)
        .filter_map(|(d, m)| m.then(|| *d))
        .collect();
    // dbg!(s_b.len());
    let s: Vec<_> = s_b
        .into_iter()
        .zip(s_t.into_iter())
        .map(|(x, y)| x - y)
        .collect();

    let file_name = format!(
        "sh48_{}-{}_calib.pkl",
        config::m1::segment::N_MODE,
        config::m1::segment::MODES
    );
    let mut m1_recon: Reconstructor = serde_pickle::from_reader(
        File::open(format!(
            "/home/ubuntu/projects/gmt-ns-im/web_server/static/main/{file_name}"
        ))?,
        Default::default(),
    )?;
    m1_recon.pseudoinverse();
    // println!("{m1_recon}");
    let imat = m1_recon.pinv_iter().nth(0).unwrap().mat_ref();
    // dbg!(imat.shape());
    let m = imat * MatRef::from_column_major_slice(&s, s.len(), 1);
    dbg!(&m);

    Ok(())
}

#[test]
fn sloperecon() -> Result<(), Box<dyn Error>> {
    type K48 = Sh48M1to2<1>;
    type AgwsSh48Frame = KernelFrame<K48>;
    let data_repo = Path::new(&env::var("DATA_REPO")?)
        .join("tests")
        .join("sh48-wavefronts");
    fs::create_dir_all(&data_repo)?;
    unsafe {
        env::set_var("DATA_REPO", data_repo);
    }

    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);

    let mut recon: ClosedLoopReconstructor = serde_pickle::from_reader(
        File::open(
            "/home/ubuntu/projects/gmt-ns-im/web_server/static/main/sh48_closed-loop_Txy_calib.pkl",
        )?,
        Default::default(),
    )?;
    recon.pseudoinverse();

    let AgwsParts {
        mut sh48,
        mut sh48_kernel,
        ..
    } = Agws::<1, 1, K48, K24>::builder()
        .gmt(gmtb.clone())
        // .sh24(ShackHartmannBuilder::sh24().use_calibration_src())
        // .sh24_calibration(sh24_recon)
        .sh48(ShackHartmannBuilder::sh48().use_calibration_src())
        .sh48_calibration(recon)
        .parts()?;
    println!("{sh48}");
    let v = sh48_kernel.processor().get_valid_lenslets().to_vec();
    let n_v = v.iter().map(|v| *v as usize).sum::<usize>();
    println!("valid lenset #: {n_v}");

    // let mut agws_slopes: gif::Frame<f64> =
    //     gif::Frame::new("agws_slopes.png", 48).image_size(256)?;
    // let mut agws_diff_slopes: gif::Frame<f64> =
    //     gif::Frame::new("agws_diff_slopes.png", 48).image_size(256)?;

    <_ as Read<M1ModeShapes>>::read(&mut sh48, m1_modes().into());
    sh48.update();
    let sh48_frame = <_ as Write<AgwsSh48Frame>>::write(&mut sh48).unwrap();
    <_ as TryRead<AgwsSh48Frame>>::try_read(&mut sh48_kernel, sh48_frame)?;
    sh48_kernel.try_update()?;
    let data = <_ as TryWrite<Estimate>>::try_write(&mut sh48_kernel)?.unwrap();

    dbg!(&data);

    // let mut data = _data.as_slice().to_vec();
    // data.chunks_mut(48 * 48 * 2)
    //     .zip(v.chunks(48 * 48))
    //     .for_each(|(data, v)| {
    //         data.chunks_mut(48 * 48).for_each(|data| {
    //             data.iter_mut().zip(v).for_each(|(data, v)| {
    //                 if *v == 0 {
    //                     *data = 0f64
    //                 }
    //             })
    //         })
    //     });

    // let min = data
    //     .iter()
    //     .min_by(|a, b| a.partial_cmp(b).unwrap())
    //     .unwrap();
    // let max = data
    //     .iter()
    //     .max_by(|a, b| a.partial_cmp(b).unwrap())
    //     .unwrap();
    // println!("Slopes min/max: ({:.3e},{:.3e})", min, max);
    // let mut w = data.chunks(48 * 48 * 2).cycle().peekable();
    // let diff_slopes: Vec<_> = (0..3)
    //     .flat_map(|_| {
    //         w.next()
    //             .unwrap()
    //             .iter()
    //             .zip(w.peek().unwrap().iter())
    //             .map(|(x, y)| {
    //                 if x.abs() > 0f64 && y.abs() > 0f64 {
    //                     x - y
    //                 } else {
    //                     0f64
    //                 }
    //             })
    //             .collect::<Vec<_>>()
    //     })
    //     .collect();
    // let min = diff_slopes
    //     .iter()
    //     .min_by(|a, b| a.partial_cmp(b).unwrap())
    //     .unwrap();
    // let max = diff_slopes
    //     .iter()
    //     .max_by(|a, b| a.partial_cmp(b).unwrap())
    //     .unwrap();
    // println!("Diff. slopes min/max: ({:.3e},{:.3e})", min, max);
    // <_ as Read<SensorData>>::read(&mut agws_slopes, data.into());
    // agws_slopes.update();
    // <_ as Read<SensorData>>::read(&mut agws_diff_slopes, diff_slopes.into());
    // agws_diff_slopes.update();

    Ok(())
}
