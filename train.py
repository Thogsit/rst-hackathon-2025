from ultralytics import YOLO, checks, hub
checks()

hub.login('b8ccc6db8d2325d86de709ab1ab6276d374f5d965f')

model = YOLO('https://hub.ultralytics.com/models/kHkI1MtBe74jM6UU8odN')
results = model.train()