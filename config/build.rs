use std::{env, path::Path};

fn main() {
    let path = env!("FEM_REPO");
    let filename = Path::new(path).file_name().unwrap().to_string_lossy();
    let result = filename
        .splitn(3, '_')
        .take(2)
        .collect::<Vec<_>>()
        .join("_");
    println!("cargo::rustc-env=FEM_SHORT_ID={result}");
}
