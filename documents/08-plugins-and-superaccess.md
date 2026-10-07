# Plugins and SuperAccess subsystem

## Purpose and scope

The plugin subsystem is the simulator’s extension mechanism. It allows external JARs and built-in modules to add runtime features without requiring a full application rebuild. In a project like EOCV-Sim, that matters because custom vision tools, extra GUI elements, and specialized integrations are often more practical as plugins than as permanent hard-coded app features.

The main implementation is spread across a few packages:

- `../VisionBench/src/main/java/org/deltacv/eocvsim/plugin/loader/PluginManager.kt`
- `../VisionBench/src/main/java/org/deltacv/eocvsim/plugin/loader/FilePluginLoaderImpl.kt`
- `../VisionBench/src/main/java/org/deltacv/eocvsim/plugin/loader/EmbeddedPluginLoader.kt`
- `../VisionBench/src/main/java/org/deltacv/eocvsim/plugin/loader/PluginClassLoader.kt`
- `../VisionBench/src/main/java/org/deltacv/eocvsim/plugin/security/superaccess/SuperAccessDaemon.kt`
- `../VisionBench/src/main/java/org/deltacv/eocvsim/plugin/security/superaccess/SuperAccessDaemonClient.kt`
- `../VisionBench/src/main/java/com/github/serivesmejia/eocvsim/plugin/api/impl/`

This subsystem is responsible for:

- finding plugin files,
- loading plugin metadata,
- enabling/disabling plugins,
- exposing a controlled API to plugins,
- and enforcing privilege approval for anything sensitive.

## Why a plugin subsystem exists at all

The app is built as a runtime platform. It has a config system, user workspace, pipeline engine, and GUI, but it also needs a way to add optional extensions without polluting the main app code and without forcing a rebuild for every feature.

That is why the plugin architecture exists.

The project does not expose raw internals freely. Instead, plugins interact via a stable API layer, which is why the API classes under `plugin/api/impl` are important: they provide the plugin contract and keep the rest of the app from being directly manipulated by arbitrary third-party code.

## `PluginManager`: the plugin lifecycle controller

`PluginManager` is the main manager for all plugin lifecycle behavior. It is effectively the top-level coordinator for the subsystem.

In `init()`, it does a sequence of setup tasks:

1. attaches a plugin output handler to the app’s UI lifecycle,
2. scans the plugin folder for JAR files,
3. creates file-based loaders for each plugin JAR,
4. loads the plugin metadata from each plugin definition,
5. checks duplicates and embedded plugins,
6. calls `loadPlugins()` to bring the plugins into the runtime,
7. enables them after the rest of the app is ready.

The manager also tracks:

- plugin files found in the plugin folder,
- plugin loaders that have been created,
- plugin source type (`FILE`, `EMBEDDED`),
- loaded plugin hashes to detect duplicates,
- and output messages shown to the user while the plugin system is active.

This makes the plugin system more than a thin jar loader. It is a runtime management layer.

## Plugin sources and loader model

The app distinguishes among plugin sources:

- `FILE` plugins — JARs located in the plugin folder
- `EMBEDDED` plugins — plugins that ship with the app itself

This classification matters for both user experience and security. A file plugin may be user-managed, while an embedded plugin is trusted as part of the app distribution.

### `FilePluginLoaderImpl`

This loader handles normal plugin JAR files. It resolves plugin info from the plugin artifact and registers the plugin with the runtime loader system.

### `EmbeddedPluginLoader`

This is used for built-in plugins that are packaged with the application. It allows the app to include optional features without forcing them to be installed manually.

### `PluginClassLoader`

A plugin class loader is necessary because plugins are external code and should not be loaded in the same classpath as the base app. This provides isolation and prevents plugin code from accidentally clashing with app classes or each other.

This is one of the most important engineering decisions in the subsystem: plugin code is treated as a separate runtime unit.

## Local plugin loading

VisionBench loads plugins directly from the local plugin folder and from bundled embedded plugins. There is no remote repository resolution layer and no dynamic download path in the app runtime.

The app only accepts plugin JARs that are present on disk or shipped with the application. This keeps plugin installation explicit and avoids untrusted third-party downloads at runtime.

## Plugin API surface

The app does not expose the entire application runtime to every plugin. Instead, plugins interact through a controlled API set. Some of the key api implementations are:

- `EOCVSimApiImpl`
- `VisualizerApiImpl`
- `PipelineManagerApiImpl`
- `InputSourceApisImpl`
- `EventHandlerHookApiImpl`
- `DialogFactoryApiImpl`
- `ConfigApiImpl`

This is the plugin contract. It allows plugin code to talk to the simulator in a stable manner without taking over the entire application. The benefit is cleaner separation of concerns: plugins work with explicit APIs instead of direct references to app internals.

## Security design and SuperAccess

This project treats plugins as potentially untrusted code. The system’s response is not “load everything unconditionally,” but rather “restrict by default and ask for permission when more power is needed.”

The key security mechanism is `SuperAccess`.

### `SuperAccessDaemon`

This is a separate JVM process that handles security-sensitive requests from plugins. It listens on a websocket connection and mediates plugin access checks. It stores granted access in a local file and validates whether a plugin is already allowed before asking the user again.

The daemon handles two kinds of security events:

- `Request` — a plugin asks for elevated privileges or access
- `Check` — the client asks whether a given plugin already has granted access

This is a system-level approval flow, not just a UI checkbox. The actual trust decision is made outside the main app process.

### `SuperAccessDaemonClient`

The client side of the daemon is the runtime component that connects to the daemon and sends grant/check requests. It is the bridge between the main app and the privileged background process.

### `SuperAccessRequest`

When a plugin requests elevated access, the app shows a dialog asking the user to approve or deny the request. The dialog includes contextual warnings and security messaging, so the user is aware that the plugin is asking for broad privileges.

This is a critical UI workflow because it makes the trust boundary explicit.

## Authority and trust validation

The code also includes authority-fetching and signature validation logic. The daemon can inspect the plugin’s declared author and verify whether a trusted authority matches the plugin signature. That means the app can distinguish between:

- a plugin signed by a trusted authority,
- a plugin claiming to be from a given author but not actually signed,
- and a plugin with no trustworthy provenance.

This is a major part of the security story and a significant difference between a toy plugin loader and a real extension platform.

## Why the trust model is so important

EOCV-Sim runs user-provided code. That is not a trivial risk model. Even though plugin authors are likely well-intentioned, they are still executing code in the same Java process as the application. If the app silently allows broad access, the extension mechanism becomes a vector for data loss or system compromise.

The project’s response is to keep the default trust model restricted and to require explicit user approval for anything beyond the sandboxed access of a normal plugin. This is exactly what a robust plugin system should do.

## Plugin output and runtime feedback

The plugin system also has a user-facing output channel through `PluginOutputHandler`. This is important because plugins often need logs, update messages, failure diagnostics, or permission warnings. Without a dedicated output surface, plugin behavior would be opaque and difficult to debug.

The design is to provide plugin output as part of the same application UX rather than as an invisible side effect. That makes plugin lifecycle events visible to the user and reduces confusion.

## Relationship to the app runtime

The plugin subsystem does not sit in isolation. It integrates with the rest of the application at several points:

- it attaches to startup and lifecycle events,
- it can use the visualizer API,
- it can interact with config values,
- it can react to pipeline and source events,
- it can show dialogs and user notifications.

This means the plugin layer is not a separable side feature. It is woven into the app’s runtime model and tooling experience.

## Why this subsystem matters

The plugin system is one of the most advanced parts of the app because it is both a developer extension mechanism and a security-sensitive runtime boundary. It allows optional features to be added without cluttering core code, while also preventing the app from turning into an unrestricted execution environment.

That balance is exactly what makes the subsystem valuable: it gives the project extensibility and safety at the same time.
