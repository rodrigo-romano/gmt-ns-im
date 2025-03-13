use std::path::Path;

use crseo::{Atmosphere, Builder, FromBuilder, RayTracing};
use skyangle::Conversion;

fn main() -> anyhow::Result<()> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("atmosphere.bin");
    let _atm = Atmosphere::builder()
        .ray_tracing(
            RayTracing::default()
                .n_width_px(865)
                .field_size(10f64.from_arcmin())
                .duration(30f64)
                .filepath(path.as_os_str())
                .n_duration(1),
        )
        .build()?;
    Ok(())
}
