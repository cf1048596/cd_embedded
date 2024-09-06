#![allow(dead_code)]
use clap::{Arg, Command};
use std::io;
use std::io::prelude::*;
use std::fs::File;
use std::io::SeekFrom;

mod instructions;



pub enum SegmentType {
    Text,
    Data,
    Bss,
    NumSegments,
}

pub enum ReferenceType {
    GlobalData,
    GlobalText,
    GlobalBss,
    TextLabelRef,
    DataLabelRef,
    BssLabelRef,
    ExternalRef,
}

pub enum InsnDescriptor {
    Insn,
    Itype,
    Rtype,
    Jtype,
    Directive,
    Other,
} 

pub struct InsnType<'a> {
    mnemonic: &'a str,
    operands: &'a str,
    op_code: u8,
    func: u8,
    insn_type_descriptor : InsnDescriptor,
}

pub struct ObjectHeader {
    magic_number : u8,
    text_seg_size : u8,
    data_seg_size : u8,
    bss_seg_size : u8,
    num_references : u8,
    symbol_name_table_size : u8,
}

pub struct RelocEntry {
    address : u8,
    symbol_ptr : u8,
    ref_type : ReferenceType,
    source_seg : SegmentType,
}

pub struct LabelEntry<'a> {
    name: Option<&'a str>,
    address: u8,
    seg_type : SegmentType,
    resolved : bool,
    is_globa : bool,
    label_entry : Option<&'a mut LabelEntry<'a>>,
    file_no : u8,

}

pub struct Reference<'a> {
    label : Option<&'a mut LabelEntry<'a>>,
    source_seg : SegmentType,
    target_seg : SegmentType,
    address : u8,
}

pub struct FileType<'a> {
    file_name : &'a str,
    segments : {
        let segments : Vec<&'a u8> = Vec::new();
        segments
    }
}

static OBJ_MAGIC_NUM : i32 = 0xdaa1; 

fn main() -> io::Result<()> {

    let label_entry_list : Vec<&mut LabelEntry<>> = Vec::new();
    let matches = Command::new("wobj")
        .version("1.0")
        .about("Inspect and optionally disassemble a wramp object file")
        .arg(
            Arg::new("FILE")
                .help("The object file to be inspected")
                .required(true)
                .index(1),
        )
        .arg(
            Arg::new("disassemble")
                .short('d')
                .long("disassemble")
                .help("Display disassembly"),
        )
        .get_matches();
      // Get the file argument
    let file = matches.get_one::<String>("FILE").unwrap();
    println!("Inspecting file: {}", file);

    // Check for the disassemble flag
    if matches.get_flag("disassemble") {
        println!("Disassembly option selected.");
    }

    let mut f = File::open(file)?;
    Ok(())
}
