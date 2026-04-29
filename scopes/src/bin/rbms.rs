use std::{
    io::{self, Write},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::Duration,
};

use crossterm::{
    event::{self, Event, KeyCode, KeyEventKind},
    terminal::{disable_raw_mode, enable_raw_mode},
};
use gmt_dos_clients_io::{gmt_m1::M1RigidBodyMotions, gmt_m2::M2RigidBodyMotions};
use gmt_dos_clients_scope_client::GridScope;
use tokio::sync::broadcast;
use gmt_dos_clients_optics_state::units::NmMas;

const RBMS: [&str; 6] = ["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"];

fn compute_hiddens(seg: Option<usize>, rbm: Option<usize>) -> Vec<String> {
    (1..=7_usize)
        .flat_map(|i| {
            RBMS.iter().enumerate().filter_map(move |(ri, r)| {
                let hide = seg.is_some_and(|s| s != i) || rbm.is_some_and(|rb| rb != ri);
                hide.then(|| format!("S{i} {r}"))
            })
        })
        .collect()
}

fn print_state(seg: Option<usize>, rbm_idx: Option<usize>) {
    let seg_str = seg.map_or("All".to_string(), |i| format!("S{i}"));
    let rbm_str = rbm_idx.map_or("All".to_string(), |i| RBMS[i].to_string());
    print!(
        "\r\x1b[2KSegment: {seg_str:<4}  RBM: {rbm_str:<3}  \
         [1-7: segment  ↑↓: rbm  0/Esc: clear all  q: quit]"
    );
    io::stdout().flush().unwrap();
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let legends: Vec<_> = (1..=7)
        .flat_map(|i| RBMS.map(|rbm| format!("S{i} {rbm}")))
        .collect();

    let (tx, rx) = broadcast::channel::<Vec<String>>(8);
    let stop = Arc::new(AtomicBool::new(false));
    let stop_clone = stop.clone();

    // Initial state: show only S1
    tx.send(compute_hiddens(Some(1), None))?;

    let grid = GridScope::new((2, 1))
        .pin_with_legends::<NmMas<M1RigidBodyMotions>>((0, 0), &legends, rx)?
        .pin_with_legends::<NmMas<M2RigidBodyMotions>>((1, 0), &legends, tx.subscribe())?;

    let repaint_ctx = grid.egui_ctx();

    let tx_clone = tx.clone();
    let h = thread::spawn(move || {
        enable_raw_mode().unwrap();

        let mut seg: Option<usize> = Some(1);
        let mut rbm_idx: Option<usize> = None;
        print_state(seg, rbm_idx);

        loop {
            if stop_clone.load(Ordering::Relaxed) {
                break;
            }
            if !event::poll(Duration::from_millis(100)).unwrap_or(false) {
                continue;
            }
            let Ok(Event::Key(key)) = event::read() else {
                continue;
            };
            if key.kind != KeyEventKind::Press {
                continue;
            }

            let changed = match key.code {
                KeyCode::Char('q') => break,
                KeyCode::Esc => {
                    seg = None;
                    rbm_idx = None;
                    true
                }
                KeyCode::Char('0') => {
                    seg = None;
                    true
                }
                KeyCode::Char(c @ '1'..='7') =>{
                    let i = (c as usize) - ('0' as usize);
                    seg = if seg == Some(i) { None } else { Some(i) };
                    true
                }
                KeyCode::Up => {
                    rbm_idx = Some(rbm_idx.map_or(0, |i| (i + 5) % 6));
                    true
                }
                KeyCode::Down => {
                    rbm_idx = Some(rbm_idx.map_or(0, |i| (i + 1) % 6));
                    true
                }
                _ => false,
            };

            if changed {
                let _ = tx_clone.send(compute_hiddens(seg, rbm_idx));
                if let Some(ctx) = repaint_ctx.get() {
                    ctx.request_repaint();
                }
                print_state(seg, rbm_idx);
            }
        }

        disable_raw_mode().unwrap();
        println!();
    });

    grid.show();

    stop.store(true, Ordering::Relaxed);
    h.join().unwrap();
    Ok(())
}
