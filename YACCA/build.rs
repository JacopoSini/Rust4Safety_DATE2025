use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
extern crate cc;

fn main() {
    cc::Build::new()
        .file("src/crt0.s")
        .compiler("riscv32-unknown-elf-gcc")
        .flag("-march=rv32i")
        .compile("crt0.a");
    
    println!("cargo:rerun-if-changed=src/crt0.s");

    println!("cargo:rerun-if-changed=linker.ld");

    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let mut f = File::create(out.join("./link.x")).unwrap();
    f.write_all(include_bytes!("linker.ld")).unwrap();
    println!("cargo:ruscc-link-search={}",out.display());
    println!("cargo:rustc-link-arg=-Tlink.x");
}