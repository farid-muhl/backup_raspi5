import argparse
import time

import matplotlib.pyplot as plt
import numpy as np

import config
from lidar_nav import LidarNavigator


MIN_RANGE_M = 0.05


def create_plot(max_range_m):
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-max_range_m, max_range_m)
    ax.set_ylim(-max_range_m, max_range_m)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (meter)")
    ax.set_ylabel("Y (meter)")
    ax.set_title("YDLIDAR YDS2MD-1 - Live POV Scan")
    ax.grid(True, alpha=0.2)

    for radius in np.arange(1, max_range_m + 1, 1):
        ax.add_patch(
            plt.Circle((0, 0), radius, fill=False, color="gray", alpha=0.35)
        )

    ax.plot(0, 0, "r+", markersize=12)
    points_plot = ax.scatter([], [], s=5, c="blue")
    status_text = ax.text(
        0.02, 0.98, "Menunggu data scan...",
        transform=ax.transAxes, va="top"
    )
    fig.tight_layout()
    return fig, points_plot, status_text


def main():
    parser = argparse.ArgumentParser(
        description="Tampilkan scan POV live dari YDLIDAR YDS2MD-1."
    )
    parser.add_argument(
        "--port",
        default=config.LIDAR_PORT,
        help=f"Port serial LiDAR (default: {config.LIDAR_PORT})",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=8.0,
        help="Jarak maksimum tampilan dalam meter (default: 8)",
    )
    args = parser.parse_args()

    if args.max_range <= MIN_RANGE_M:
        parser.error(f"--max-range harus lebih besar dari {MIN_RANGE_M} meter")

    lidar = None
    fig = None
    try:
        lidar = LidarNavigator(port=args.port, baud=config.LIDAR_BAUD)
        if not lidar.connected:
            raise OSError(
                f"Tidak dapat membuka YDLIDAR pada {args.port}; "
                "periksa port, kabel, dan izin akses."
            )

        fig, points_plot, status_text = create_plot(args.max_range)
        print(
            f"[LIDAR] Membaca YDLIDAR YDS2MD-1 dari {args.port} "
            f"pada {config.LIDAR_BAUD} baud."
        )
        print("Tutup jendela grafik atau tekan Ctrl+C untuk berhenti.")

        last_update_time = time.monotonic()
        last_scan_time = lidar.last_scan_time
        while plt.fignum_exists(fig.number):
            lidar.update_scan()

            if lidar.scan_data:
                scan = np.asarray(
                    list(lidar.scan_data.items()), dtype=float
                )
                angles_rad = np.deg2rad(scan[:, 0])
                distances_m = scan[:, 1] / 100.0
                valid = (
                    np.isfinite(angles_rad)
                    & np.isfinite(distances_m)
                    & (distances_m > MIN_RANGE_M)
                    & (distances_m <= args.max_range)
                )

                distances_m = distances_m[valid]
                angles_rad = angles_rad[valid]
                x = distances_m * np.cos(angles_rad)
                y = distances_m * np.sin(angles_rad)
                points_plot.set_offsets(np.column_stack((x, y)))

                if lidar.last_scan_time != last_scan_time:
                    last_scan_time = lidar.last_scan_time
                    last_update_time = time.monotonic()
                status_text.set_text(
                    f"Titik: {len(x)} | "
                    f"Jangkauan: {args.max_range:.1f} m | "
                    f"Data terakhir: {time.monotonic() - last_update_time:.1f} s"
                )
            else:
                status_text.set_text("Belum menerima paket scan YDLIDAR")

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.01)

    except KeyboardInterrupt:
        print("\n[LIDAR] Tes dihentikan.")
    except (OSError, ValueError) as exc:
        print(f"[LIDAR] Gagal membaca YDLIDAR: {exc}")
        return 1
    finally:
        if lidar is not None:
            try:
                lidar.close()
            except (OSError, ValueError) as exc:
                print(f"[LIDAR] Gagal menutup koneksi: {exc}")

        plt.ioff()
        if fig is not None:
            plt.close(fig)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
