use gmt_dos_clients_io::optics::{SegmentPiston, SegmentTipTilt, SegmentWfeRms, TipTilt, WfeRms};
use gmt_dos_clients_scope_client::GridScope;
use interface::units::Mas;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    loop {
        GridScope::new((3, 1))
            // .n_sample(5_000)
            .pin::<WfeRms<-9>>((0, 0))?
            .pin::<SegmentWfeRms<-9>>((0, 0))?
            .pin::<Mas<TipTilt>>((1, 0))?
            .pin::<Mas<SegmentTipTilt>>((1, 0))?
            .pin::<SegmentPiston<-9>>((2, 0))?
            .show();
    }
    Ok(())
}
