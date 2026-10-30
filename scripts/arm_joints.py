"""Inspect and save leader/follower zero and reference poses with Rerun sliders."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from data_pipeline.data_rerun.arm_pose import (
    ARMS, POSES, DEFAULT_REFERENCE, DEFAULT_ROBOT_XACRO, JOINT_NAMES,
    ArmModel, ArmPosePreview, MappingPreview, load_pose, pose_identity, save_pose, save_document, validate_angles,
)
from data_pipeline.data_collection.teleoperation.mapping import JointMapping


class SliderWindow:
    def __init__(self, root, preview, angles, output, overwrite=False, directions=None):
        import tkinter as tk
        from tkinter import ttk

        self.root, self.preview, self.output = root, preview, output
        self.overwrite = overwrite
        self.is_mapping = isinstance(preview, MappingPreview)
        self.probe = None
        self.directions = []
        directions = [1] * 6 if directions is None else directions
        self.dirty = True
        self.pending = None
        self.values, self.entries = [], []
        root.title('Absolute Leader/Follower Mapping' if self.is_mapping else f'{preview.model.label} - {preview.purpose.title()} Pose Controls')
        width, height = 460, min(740, root.winfo_screenheight() - 100)
        root.geometry(f'{width}x{height}+{max(0, root.winfo_screenwidth() - width - 20)}+40')
        root.minsize(400, 580)
        root.attributes('-topmost', True)
        root.protocol('WM_DELETE_WINDOW', self.close)
        frame = ttk.Frame(root, padding=16)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='Leader / Follower Mapping' if self.is_mapping else f'{preview.model.label} - {preview.purpose.title()}', font=('', 17, 'bold')).pack(anchor='w')
        ttk.Label(frame, text=('Leader fixed at zero. Sliders set follower offsets.\nSelect each joint sign; +10 deg buttons check direction.' if self.is_mapping else 'Offline editor - no robot connection\nAngles are relative to the current URDF zero.')).pack(anchor='w', pady=(6, 10))
        for index, (value, (lo, hi)) in enumerate(zip(angles, preview.limits_deg)):
            if not lo <= value <= hi:
                raise ValueError(f'{JOINT_NAMES[index]} is outside the URDF limits.')
            group = ttk.Frame(frame)
            group.pack(fill='x', pady=6)
            header = ttk.Frame(group)
            header.pack(fill='x')
            ttk.Label(header, text=f'joint_{index+1}', font=('', 11, 'bold')).pack(side='left')
            if self.is_mapping:
                sign = tk.StringVar(value=f'{int(directions[index]):+d}')
                self.directions.append(sign)
                selector = ttk.Combobox(header, textvariable=sign, values=('+1', '-1'), width=3, state='readonly')
                selector.pack(side='left', padx=8)
                selector.bind('<<ComboboxSelected>>', self.mark_dirty)
                ttk.Button(header, text='+10 deg', width=8, command=lambda i=index: self.toggle_probe(i)).pack(side='left')
            text = tk.StringVar(value=str(value))
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
        ttk.Button(buttons, text='Copy Mapping' if self.is_mapping else 'Copy Angles', command=self.copy).pack(side='left', padx=8)
        ttk.Button(buttons, text='Save Mapping...' if self.is_mapping else 'Save Pose...', command=self.save).pack(side='right')
        self.status = tk.StringVar(value='Drag a slider or type an angle and press Enter.')
        ttk.Label(frame, textvariable=self.status, wraplength=415).pack(anchor='w', pady=6)
        ttk.Label(frame, text=('Absolute: follower = offset + direction * leader\nDirection checks never change the saved zero offsets.' if self.is_mapping else 'Save Pose records joint angles for this arm and purpose.\nIt does not modify the URDF or calibrate hardware.'), wraplength=415).pack(anchor='w', pady=6)
        self.pending = root.after(30, self.refresh)

    def mark_dirty(self, event=None):
        self.dirty = True

    def toggle_probe(self, index):
        self.probe = None if self.probe == index else index
        self.dirty = True
        self.status.set('Leader at zero.' if self.probe is None else f'Checking joint_{index+1} at +10 deg. Click again to return to zero.')

    def update_preview(self, angles):
        if self.is_mapping:
            self.preview.update_mapping(angles, [int(v.get()) for v in self.directions], self.probe)
        else:
            self.preview.update(angles)

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
        self.probe = None
        self.status.set('Offsets reset to zero; directions preserved.' if self.is_mapping else 'All joints reset to the current URDF zero.')

    def copy(self):
        angles = self.angles()
        if angles is None:
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(json.dumps({'offset_deg': angles, 'direction': [int(v.get()) for v in self.directions]} if self.is_mapping else angles))
        self.status.set('Mapping parameters copied to the clipboard.' if self.is_mapping else 'Joint angles copied to the clipboard.')

    def save(self):
        from tkinter import filedialog, messagebox

        angles = self.angles()
        if angles is None:
            return
        filename = filedialog.asksaveasfilename(
            parent=self.root, title='Save Absolute Mapping' if self.is_mapping else f'Save {self.preview.model.label} {self.preview.purpose.title()} Pose', defaultextension='.json',
            filetypes=[('JSON', '*.json')], initialdir=str(self.output.parent.resolve()),
            initialfile=self.output.name, confirmoverwrite=False,
        )
        if not filename:
            return
        try:
            # Apply the exact values being saved to the preview first.
            self.update_preview(angles)
            self.dirty = False
            if self.is_mapping:
                replaced = save_document(filename, self.preview.current_mapping.document, overwrite=self.overwrite)
            else:
                replaced = save_pose(filename, angles, self.preview.model, self.preview.purpose, overwrite=self.overwrite)
        except (OSError, ValueError) as exc:
            messagebox.showerror('Save Failed', str(exc), parent=self.root)
            return
        self.output = Path(filename)
        self.status.set(f'Saved: {filename}')
        print(f'覆盖 {filename}' if replaced else f'Saved {self.preview.model.arm} {self.preview.purpose} pose: {filename}', flush=True)

    def refresh(self):
        from tkinter import messagebox

        self.pending = None
        if not self.preview.is_open():
            self.close()
            return
        if self.dirty:
            try:
                self.update_preview([v.get() for v in self.values])
            except ValueError as exc:
                self.status.set(f'Preview rejected: {exc}')
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


def joint_angles(value):
    try:
        return validate_angles(json.loads(value))
    except (ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError('Use a JSON array of six finite angles in degrees.') from exc


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--calibrate_leader_follwer', '--calibrate-leader-follower', dest='calibrate_leader_follower', action='store_true', help='Offline absolute mapping: leader held at zero, edit follower offsets and per-joint signs')
    parser.add_argument('--arm', choices=ARMS, help='Robot to edit; defaults to input-file arm or leader')
    parser.add_argument('--pose', choices=POSES, help='Pose purpose; defaults to input-file type or zero')
    initial = parser.add_mutually_exclusive_group()
    initial.add_argument('--input', type=Path, help='Load saved angles and infer arm/pose unless explicitly specified')
    initial.add_argument('--joints', type=joint_angles, help='Initial joint angles in degrees, e.g. "[90,90,-90,0,0,90]"')
    parser.add_argument('--output', type=Path, help='Default save filename; configs/teleoperation/<arm>.<pose>.json')
    parser.add_argument('--overwrite', action='store_true', help='Allow replacing an existing pose file when saving')
    parser.add_argument('--robot-xacro', type=Path, default=DEFAULT_ROBOT_XACRO, help='ECO63 xacro used for --arm follower')
    parser.add_argument('--memory-limit', default='25%')
    args = parser.parse_args()
    directions = None
    leader_model = None
    if args.calibrate_leader_follower:
        if args.arm or args.pose or args.joints is not None:
            parser.error('--calibrate_leader_follwer cannot be combined with --arm, --pose, or --joints')
        output = args.output or Path('configs/teleoperation/leader_follower.mapping.json')
        if output.exists() and not args.overwrite:
            raise FileExistsError(f'{output} already exists. Use --overwrite to replace it.')
        leader_model, model = ArmModel.load('leader'), ArmModel.load('follower', args.robot_xacro)
        purpose = 'mapping'
        mapping = (JointMapping.load(args.input, leader_model.sha256, model.sha256) if args.input else
                   JointMapping.create([0.] * 6, [1] * 6, leader_model.sha256, model.sha256))
        angles, directions = mapping.offset_deg.tolist(), mapping.direction.tolist()
    else:
        inferred_arm, inferred_pose = pose_identity(json.loads(args.input.read_text())) if args.input else ('leader', 'zero')
        arm, purpose = args.arm or inferred_arm, args.pose or inferred_pose
        if args.input and (arm, purpose) != (inferred_arm, inferred_pose):
            parser.error('The requested --arm / --pose does not match the input pose file.')
        output = args.output or Path('configs/teleoperation') / f'{arm}.{purpose}.json'
        if output.exists() and not args.overwrite:
            raise FileExistsError(f'{output} already exists. Use --overwrite to replace it.')
        model = ArmModel.load(arm, args.robot_xacro)
        angles = (load_pose(args.input, model, purpose) if args.input else
                  args.joints if args.joints is not None else
                  DEFAULT_REFERENCE[arm] if purpose == 'reference' else (0.,) * 6)
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
        preview = (MappingPreview(leader_model, model, args.memory_limit) if args.calibrate_leader_follower
                   else ArmPosePreview(model, purpose, args.memory_limit))
        SliderWindow(root, preview, angles, output, overwrite=args.overwrite, directions=directions)
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
