use crseo::{Atmosphere, Builder, FromBuilder, RayTracing};
use skyangle::Conversion;

fn main() -> anyhow::Result<()> {
    let _atm = Atmosphere::builder()
        .ray_tracing(
            RayTracing::default()
                .n_width_px(865)
                .field_size(10f64.from_arcmin())
                .duration(30f64)
                .filepath("atmosphere.bin")
                .n_duration(1),
        )
        .build()?;
    Ok(())
}
