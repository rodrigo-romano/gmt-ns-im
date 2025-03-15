use actix_files as fs;
use actix_web::{App, HttpServer, Result, web};
use std::{env, path::PathBuf};

// async fn index() -> Result<fs::NamedFile> {
//     Ok(fs::NamedFile::open(PathBuf::from("static/index.html"))?)
// }

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    let address = if let Ok(ip) = env::var("SCOPE_SERVER_IP") {
        ip
    } else {
        "127.0.0.1".to_string()
    };
    println!("Starting server at http://{address}:8888");

    HttpServer::new(|| {
        App::new()
            // Route for the root path
            // .route("/", web::get().to(index))
            // Serve static files from the "static" directory
            .service(fs::Files::new("/static", "static").show_files_listing())
    })
    .bind(format!("{address}:8888"))?
    .run()
    .await
}
