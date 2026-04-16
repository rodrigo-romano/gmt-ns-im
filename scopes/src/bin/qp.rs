#![allow(unreachable_code)]

use std::env;

use gmt_dos_clients_io::gmt_m1::{self, segment::ModeShapes};
use gmt_dos_clients_optics_state::MirrorState;
use gmt_dos_clients_scope_client::Scope;
use interface::{Data, UniqueIdentifier, Write};

pub enum M1RBM<const ID: u8> {}
impl<const ID: u8> UniqueIdentifier for M1RBM<ID> {
    type DataType = <gmt_m1::segment::RBM<ID> as UniqueIdentifier>::DataType;
    const PORT: u16 = 51_110 + ID as u16;
}
impl<const ID: u8> Write<M1RBM<ID>> for MirrorState {
    fn write(&mut self) -> Option<Data<M1RBM<ID>>> {
        <_ as Write<gmt_m1::segment::RBM<ID>>>::write(self).map(|data| data.transmute())
    }
}

pub enum M2RBM<const ID: u8> {}
impl<const ID: u8> UniqueIdentifier for M2RBM<ID> {
    type DataType = <gmt_m1::segment::RBM<ID> as UniqueIdentifier>::DataType;
    const PORT: u16 = 52_220 + ID as u16;
}
impl<const ID: u8> Write<M2RBM<ID>> for MirrorState {
    fn write(&mut self) -> Option<Data<M2RBM<ID>>> {
        <_ as Write<gmt_m1::segment::RBM<ID>>>::write(self).map(|data| data.transmute())
    }
}

const SID: u8 = 1;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing::subscriber::set_global_default(
        tracing_subscriber::FmtSubscriber::builder()
            .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
            .finish(),
    )?;
    loop {
        match env::var("GMT")?.as_str() {
            "M1" => {
                Scope::new().name("M1 RBMS").signal::<M1RBM<SID>>()?.show();
            }
            "M1=modes" => {
                Scope::new()
                    .name("M1 Bending Modes")
                    .signal::<ModeShapes<SID>>()?
                    .show();
            }
            "M2" => {
                Scope::new().name("M2 RBMS").signal::<M2RBM<SID>>()?.show();
            }
            _ => panic!("GMT env should be set to M1 or M2"),
        }
    }
    Ok(())
}
