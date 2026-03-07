pub fn now() -> f64 {
    use std::time::SystemTime;

    let time = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap_or_else(|e| panic!("{}", e));
    time.as_secs_f64()
}