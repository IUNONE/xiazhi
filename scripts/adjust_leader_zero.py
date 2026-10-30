"""Edit the U-Arm zero pose using a separate slider window beside Rerun."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from data_pipeline.data_rerun.leader_zero import JOINT_NAMES, ZeroPosePreview, load_pose, save_pose


class SliderWindow:
    def __init__(self, root, preview, angles, output, overwrite=False):
        import tkinter as tk
        from tkinter import ttk

        self.root, self.preview, self.output = root, preview, output
        self.overwrite = overwrite
        self.dirty = True
        self.pending = None
        self.values, self.entries = [], []
        root.title('U-Arm Zero Pose Controls')
        width, height = 460, min(740, root.winfo_screenheight() - 100)
        root.geometry(f'{width}x{height}+{max(0, root.winfo_screenwidth() - width - 20)}+40')
        root.minsize(400, 580)
        root.attributes('-topmost', True)
        root.protocol('WM_DELETE_WINDOW', self.close)
        frame = ttk.Frame(root, padding=16)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='U-Arm Zero Pose', font=('', 17, 'bold')).pack(anchor='w')
        ttk.Label(frame, text='Offline editor - no robot connection\nAngles are relative to the current URDF zero.').pack(anchor='w', pady=(6, 10))
        for index, (value, (lo, hi)) in enumerate(zip(angles, preview.limits_deg)):
            if not lo <= value <= hi:
                raise ValueError(f'{JOINT_NAMES[index]} is outside the URDF limits.')
            group = ttk.Frame(frame)
            group.pack(fill='x', pady=6)
            header = ttk.Frame(group)
            header.pack(fill='x')
            ttk.Label(header, text=f'joint_{index+1}', font=('', 11, 'bold')).pack(side='left')
            text = tk.StringVar(value=f'{value:.1f}')
            variable = tk.DoubleVar(value=value)
            self.entries.append(text)
            self.values.append(variable)
            ttk.Label(header, text='deg').pack(side='right', padx=(4, 0))
            entry = ttk.Entry(header, textvariable=text, width=10, justify='right')
            entry.pack(side='right')
            entry.bind('<Return>', lambda event, i=index: self.apply_entry(i))
            entry.bind('<FocusOut>', lambda event, i=index: self.apply_entry(i))
            ttk.Scale(group, from_=lo, to=hi, variable=variable,
                      command=lambda value, i=index: self.slide(i, value)).pack(fill='x', pady=(4, 0))
        buttons = ttk.Frame(frame)
        buttons.pack(fill='x', pady=(12, 6))
        ttk.Button(buttons, text='Reset All', command=self.reset).pack(side='left')
        ttk.Button(buttons, text='Copy Angles', command=self.copy).pack(side='left', padx=8)
        ttk.Button(buttons, text='Save Pose...', command=self.save).pack(side='right')
        self.status = tk.StringVar(value='Drag a slider or type an angle and press Enter.')
        ttk.Label(frame, textvariable=self.status, wraplength=415).pack(anchor='w', pady=6)
        ttk.Label(frame, text='Save the pose JSON and share it to update the URDF.\nThis file is separate from encoder calibration.', wraplength=415).pack(anchor='w', pady=6)
        self.pending = root.after(30, self.refresh)

    def slide(self, index, value):
        value = round(float(value), 1)
        self.values[index].set(value)
        self.entries[index].set(f'{value:.1f}')
        self.dirty = True

    def apply_entry(self, index):
        lo, hi = self.preview.limits_deg[index]
        try:
            value = float(self.entries[index].get())
            if not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError
        except ValueError:
            self.status.set(f'joint_{index+1}: enter an angle from {lo:.1f} to {hi:.1f} deg.')
            return False
        self.values[index].set(value)
        self.dirty = True
        return True

    def angles(self):
        if not all(self.apply_entry(i) for i in range(6)):
            return None
        return [v.get() for v in self.values]

    def reset(self):
        for i in range(6):
            self.slide(i, 0)
        self.status.set('All joints reset to the current URDF zero.')

    def copy(self):
        angles = self.angles()
        if angles is None:
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(json.dumps(dict(zip(JOINT_NAMES, angles))))
        self.status.set('Joint angles copied to the clipboard.')

    def save(self):
        from tkinter import filedialog, messagebox

        angles = self.angles()
        if angles is None:
            return
        filename = filedialog.asksaveasfilename(
            parent=self.root, title='Save Zero Pose', defaultextension='.json',
            filetypes=[('JSON', '*.json')], initialdir=str(self.output.parent.resolve()),
            initialfile=self.output.name, confirmoverwrite=False,
        )
        if not filename:
            return
        try:
            # Apply the exact values being saved to the preview first.
            self.preview.update(angles)
            self.dirty = False
            replaced = save_pose(filename, angles, overwrite=self.overwrite)
        except (OSError, ValueError) as exc:
            messagebox.showerror('Save Failed', str(exc), parent=self.root)
            return
        self.output = Path(filename)
        self.status.set(f'Saved: {filename}')
        print(f'覆盖 {filename}' if replaced else f'Saved zero pose: {filename}', flush=True)

    def refresh(self):
        from tkinter import messagebox

        self.pending = None
        if not self.preview.is_open():
            self.close()
            return
        if self.dirty:
            try:
                self.preview.update([v.get() for v in self.values])
            except Exception as exc:
                messagebox.showerror('Preview Error', str(exc), parent=self.root)
                self.close()
                return
            self.dirty = False
        self.pending = self.root.after(30, self.refresh)

    def close(self):
        if self.pending is not None:
            self.root.after_cancel(self.pending)
            self.pending = None
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, help='Load a saved zero-pose JSON')
    parser.add_argument('--output', type=Path, default=Path('uarm.zero-pose.json'), help='Default filename in the Save Pose dialog')
    parser.add_argument('--overwrite', action='store_true', help='Allow replacing an existing pose file when saving')
    parser.add_argument('--memory-limit', default='25%')
    args = parser.parse_args()
    if args.output.exists() and not args.overwrite:
        raise FileExistsError(f'{args.output} already exists. Use --overwrite to replace it.')
    angles = load_pose(args.input) if args.input else [0.] * 6
    try:
        import tkinter as tk
    except ImportError as exc:
        parser.exit(1, f'Tkinter is required. On Ubuntu run: sudo apt-get install python3-tk\n{exc}\n')
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        parser.exit(1, f'Open this editor in a graphical desktop session: {exc}\n')
    root.withdraw()
    preview = None
    try:
        preview = ZeroPosePreview(args.memory_limit)
        SliderWindow(root, preview, angles, args.output, overwrite=args.overwrite)
        root.deiconify()
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if preview is not None:
            preview.close()
        try:
            root.destroy()
        except tk.TclError:
            pass


if __name__ == '__main__':
    main()
