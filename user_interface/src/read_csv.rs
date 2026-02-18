use std::error::Error;
use std::fs::File;
use std::io;
use csv::ReaderBuilder;

pub fn read_csv(path: &str) -> Result<Vec<(f32, f32, f32, f32)>, Box<dyn Error>> {
    let file = File::open(path)?;
    let mut rdr = ReaderBuilder::new()
        .has_headers(true)
        .from_reader(file);

    let mut data = Vec::new();

    for result in rdr.records() {
        let record = result?;
        // Parse each column as f64
        let time = record[0].parse::<f32>()?;
        let x = record[1].parse::<f32>()?;
        let y = record[2].parse::<f32>()?;
        let z = record[3].parse::<f32>()?;
        data.push((time, x, y, z));
    }

    Ok(data)
}
