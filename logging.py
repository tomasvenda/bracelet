import os
import csv

def process_dump(log_path, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    
    # --- NEW: Scan existing files to find the current highest counts ---
    count = {"fall": 0, "no_fall": 0}
    for file in os.listdir(out_dir):
        if file.endswith(".csv") and file != "metadata_summary.csv":
            # Split filename (e.g., "fall.62.csv" -> ["fall", "62", "csv"])
            parts = file.split(".")
            if len(parts) >= 3:
                label, num = parts[0], parts[1]
                if label in count and num.isdigit():
                    # Update count if we find a higher number
                    count[label] = max(count[label], int(num))
    
    print(f"Found existing data. Resuming counts from: {count}\n")
    # -------------------------------------------------------------------

    rows, name = [], None
    metadata_str = ""
    summary_data = []

    def flush():
        nonlocal rows, name, metadata_str
        if name and rows:
            label = "fall" if name.startswith("fall") else "no_fall"
            count[label] = count.get(label, 0) + 1
            event_id = f"{label}.{count[label]}"
            
            # 1. Parse the metadata line into a dictionary
            meta_dict = {"event_id": event_id}
            if metadata_str:
                parts = metadata_str.split()
                for p in parts:
                    if "=" in p:
                        k, v = p.split("=")
                        meta_dict[k] = v
            summary_data.append(meta_dict)
            
            print(f"[{event_id}] {metadata_str}")
            
            # 2. Process and save the time-series sensor data
            t0 = int(rows[0].split(",")[0])
            p0 = float(rows[0].split(",")[4])
            path = os.path.join(out_dir, f"{event_id}.csv")
            
            with open(path, "w") as f:
                f.write("timestamp,acc_x,acc_y,acc_z,pressure_delta\n")
                for r in rows:
                    p = r.split(",")
                    p[0] = str(int(p[0]) - t0)
                    p[4] = f"{float(p[4]) - p0:.3f}"
                    f.write(",".join(p) + "\n")
                    
        rows, name, metadata_str = [], None, ""

    # Parse the log file
    with open(log_path) as f:
        for line in f:
            s = line.strip()
            # REMOVED the break condition here so it reads the whole file
            if s.startswith("-----") and s.endswith("-----"):
                flush()
                name = s.replace("-", "").strip()
            elif s.startswith("# ml="):
                metadata_str = s.replace("# ", "")
            elif name and s and not s.startswith(("#", "=====", "timestamp")):
                rows.append(s)
    flush()
    
    # 3. Write the aggregated metadata summary to a master CSV
    if summary_data:
        summary_path = os.path.join(out_dir, "metadata_summary.csv")
        file_exists = os.path.isfile(summary_path)
        
        headers = summary_data[0].keys()
        
        # --- NEW: Open in "a" (append) mode instead of "w" (overwrite) ---
        with open(summary_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            # Only write the header row if this is a brand new file
            if not file_exists:
                writer.writeheader()
            writer.writerows(summary_data)
        # -----------------------------------------------------------------
        
        print(f"\nAppended metadata summary to: {summary_path}")

    print(f"Extraction Complete. New Totals -> {count}")

process_dump("log.log.txt", "ml_training_data")