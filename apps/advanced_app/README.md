# advanced_app

Build and run from repo root:

cmake -S . -B build
cmake --build build --target advanced_app -- -j
./build/apps/advanced_app/advanced_app

advanced_app
============

Minimal standalone app demonstrating pose->matrix compose and point transform.

Build
-----
From repo root:

```
cmake -S . -B build
cmake --build build --target advanced_app -- -j 4
```

Run
---

```
./build/apps/advanced_app/advanced_app
```
