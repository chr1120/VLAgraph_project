import assert from "node:assert/strict";
import { test } from "node:test";
import { fileURLToPath } from "node:url";
import { build } from "esbuild";
import { JSDOM } from "jsdom";

test("workflow detail buttons open both groups after a slight mouse movement", async () => {
  const dom = new JSDOM('<div id="root"></div>', { url: "http://localhost", pretendToBeVisual: true });
  const globals = new Map();
  function setGlobal(name, value) {
    globals.set(name, Object.getOwnPropertyDescriptor(globalThis, name));
    Object.defineProperty(globalThis, name, { value, configurable: true, writable: true });
  }
  for (const name of ["window", "document", "navigator", "HTMLElement", "Element", "SVGElement", "Node", "getComputedStyle"]) {
    setGlobal(name, dom.window[name]);
  }
  setGlobal("requestAnimationFrame", dom.window.requestAnimationFrame.bind(dom.window));
  setGlobal("cancelAnimationFrame", dom.window.cancelAnimationFrame.bind(dom.window));
  // Layout is outside this test; retain React Flow's real mouse event handling.
  setGlobal("ResizeObserver", class { observe() {} unobserve() {} disconnect() {} });
  setGlobal("IS_REACT_ACT_ENVIRONMENT", true);
  // Bundled React's async act fallback creates MessagePorts; close them after use.
  const channels = [];
  const NativeMessageChannel = globalThis.MessageChannel;
  setGlobal("MessageChannel", class extends NativeMessageChannel {
    constructor() { super(); channels.push(this); }
  });
  const bundle = await build({
    stdin: {
      contents: 'export {default as Page} from "./src/pages/GraphPage"; export {default as React, act} from "react"; export {createRoot} from "react-dom/client";',
      resolveDir: fileURLToPath(new URL("..", import.meta.url)), loader: "tsx",
    },
    bundle: true, write: false, platform: "node", format: "esm", loader: { ".css": "empty" },
    define: { "import.meta.env": "{}", "process.env.NODE_ENV": '"test"' },
  });
  const { Page, React, act, createRoot } = await import(`data:text/javascript;base64,${Buffer.from(bundle.outputFiles[0].text).toString("base64")}`);
  const root = createRoot(document.getElementById("root"));
  async function click(button) {
    assert.ok(button, "expected button exists");
    await act(async () => {
      button.dispatchEvent(new window.MouseEvent("mousedown", { bubbles: true, view: window, buttons: 1, clientX: 100, clientY: 100 }));
      window.dispatchEvent(new window.MouseEvent("mousemove", { bubbles: true, view: window, buttons: 1, clientX: 101, clientY: 100 }));
      window.dispatchEvent(new window.MouseEvent("mouseup", { bubbles: true, view: window, clientX: 101, clientY: 100 }));
      button.click();
      // D3 removes its temporary click suppression on the next event loop turn.
      await new Promise(resolve => setTimeout(resolve, 1));
    });
  }
  const buttonNamed = text => [...document.querySelectorAll("button")].find(button => button.textContent === text);
  try {
    await act(async () => root.render(React.createElement(Page)));
    await click(buttonNamed("Load"));
    await click([...document.querySelectorAll(".graph-template-option")].find(button => button.textContent.includes("FANUC M-710iC/20L 실행 검증 (Full)")));
    await click(buttonNamed("덮어쓰기"));
    for (const [index, title, count] of [[0, "3. 명령 준비", 2], [1, "4. 통합 검증", 8]]) {
      const buttons = [...document.querySelectorAll("button")].filter(button => button.textContent === "상세 노드 보기");
      assert.equal(buttons.length, 2);
      await click(buttons[index]);
      const dialog = document.querySelector(".bundle-window");
      assert.ok(dialog, "detail dialog opens despite mouse movement");
      assert.equal(dialog.querySelector("h2").textContent, title);
      assert.equal(dialog.querySelectorAll(".react-flow__node").length, count);
      await click(dialog.querySelector("button"));
      assert.equal(document.querySelector(".bundle-window"), null);
    }
    assert.equal(document.querySelectorAll(".react-flow__node").length, 6);
  } finally {
    await act(async () => root.unmount());
    for (const channel of channels) { channel.port1.close(); channel.port2.close(); }
    dom.window.close();
    for (const [name, descriptor] of globals) {
      if (descriptor) Object.defineProperty(globalThis, name, descriptor);
      else delete globalThis[name];
    }
  }
});
