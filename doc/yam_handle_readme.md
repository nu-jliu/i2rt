# YAM handld basic

## Read handle output:
```python
python scripts/read_encoder.py --channel $CAN_CHANNEL
```

you should be able to see output like this:
```bash
  [PassiveEncoderInfo(id=1294, position=np.float64(0.004382802251101832), velocity=0.0, io_inputs=[0, 0])]
```

position is the trigger position, later we will map this to the gripper position. The handle also has two reserved
