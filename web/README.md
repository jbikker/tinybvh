# TinyBVH custom header, on the web

A static page that does what `tinybvh_custom` does: choose the parts of TinyBVH a project uses,
and download a single header that contains only those. It runs the same generator, compiled to
WebAssembly, entirely in the browser; nothing is uploaded, and no server code is needed.

## Files

This folder holds the page's sources; the generator and the headers it works on come from the
repository itself.

| File | What it is |
|---|---|
| `index.html` | The page. Its toggles come from the generator itself (`--describe`). |
| `generator.js` | Runs the generator on headers in the module's in-memory file system. |
| `worker.js` | Runs it in a background worker, so the page stays responsive. |
| `build_web.sh` | Builds `tinybvh_custom.js` with Emscripten, and copies the six headers into `headers/`. |
| `check_web.cjs` | Checks a built page against the native generator. |

`tinybvh_custom.js` and `headers/` are build output, made where `build_web.sh` runs; they are
not part of the repository.

## Publishing

`.github/workflows/web.yml` builds and publishes the page on every push to `main` that changes the
headers, `tinybvh_custom.cpp` or this folder, and can also be run by hand from the Actions tab. It
installs Emscripten, builds the page in `_site/`, checks it with `check_web.cjs` against the native
generator, and deploys it only if every check passes.

One-time setup: in the repository's Settings, Pages, set "Source" under "Build and deployment" to
"GitHub Actions". The page then appears at `https://<owner>.github.io/<repository>/`.

The Emscripten version is `EMSDK_VERSION` at the top of the workflow.

## Building and trying it locally

From the repository root, with Emscripten (emsdk) on the path, the same steps as the workflow:

    mkdir -p _site
    cp web/index.html web/worker.js web/generator.js _site/
    cd _site
    sh ../web/build_web.sh ../tinybvh_custom.cpp ..
    python -m http.server

Then open `http://localhost:8000/`. Opened straight from disk instead, browsers refuse to load the
bundled headers; the page then says so, and can still generate from headers chosen with "Generate
from your own copy of TinyBVH".

## Browser support

The generator uses WebAssembly's own exception handling, to report malformed headers: Chrome and
Edge 95, Firefox 100, Safari 15.2, and later.
