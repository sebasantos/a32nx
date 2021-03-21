import { ExecTask, TaskOfTasks } from '@flybywiresim/igniter';

export default new TaskOfTasks('a32nx', [
    new TaskOfTasks('build', [
        new ExecTask('instruments','npm run build:instruments', ['src/instruments', 'A32NX/html_ui/Pages/VCockpit/Instruments/generated']),
        new ExecTask('behavior','node src/behavior/build.js', ['src/behavior', 'A32NX/ModelBehaviorDefs/A32NX/generated']),
        new ExecTask('model','node src/model/build.js', ['src/model', 'A32NX/SimObjects/AirPlanes/Asobo_A320_NEO/model']),
        new ExecTask('systems', [
            'cargo build --target wasm32-wasi --release',
            'cp -u target/wasm32-wasi/release/systems.wasm A32NX/SimObjects/AirPlanes/Asobo_A320_NEO/panel/'
        ]),
    ], true),

    new TaskOfTasks('dist', [
        new ExecTask('manifests', 'node scripts/build.js'),
        new ExecTask('metadata', 'bash scripts/metadata.sh'),
    ]),
]);
