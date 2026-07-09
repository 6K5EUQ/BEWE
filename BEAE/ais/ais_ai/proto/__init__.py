"""Proprietary AIS RF-fingerprint pipeline (experimental, isolated).

Residual-extraction front-end (A) + open-set prototypical embedding (B).

Fully separate from the deployed IQResNet1D closed-set model:
  - code:      ais_ai/proto/*            (ais_ai/*.py untouched)
  - artifacts: data/ai_model_proto/*     (data/ai_model/* v1-v4 untouched)
  - CLI:       python -m ais_ai.proto    (python -m ais_ai unchanged)

Not wired to the live UDS daemon — train/eval/enroll only. Reverting is a
no-op for the running system: delete data/ai_model_proto/ and this package.
"""
