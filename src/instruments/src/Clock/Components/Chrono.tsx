import React, { useEffect, useState } from 'react';
import { useInteractionEvent } from '../../Common/hooks';
import { useSimVar } from '../../Common/simVars';

const getDisplayString = (seconds: number | null, running: boolean) : string => (seconds == null ? ''
    : `${Math.floor(Math.min(seconds, 5999) / 60).toString().padStart(2, '0')}${running ? ':' : ' '}${(Math.floor(Math.min(seconds, 5999) % 60)).toString().padStart(2, '0')}`);

export const Chrono = () => {
    const [ltsTest] = useSimVar('L:XMLVAR_LTS_Test', 'bool', 250);
    const [absTime] = useSimVar('E:ABSOLUTE TIME', 'Seconds', 200);
    const [prevTime, setPrevTime] = useState(absTime);

    const [elapsedTime, setElapsedTime] = useState<null | number>(null);
    const [running, setRunning] = useState(false);

    useEffect(() => {
        if (running) {
            setElapsedTime(Math.max((elapsedTime || 0) + absTime - prevTime, 0));
        }
        setPrevTime(absTime);
    }, [absTime]);

    useInteractionEvent('A32NX_CHRONO_TOGGLE', () => {
        if (running) {
            setRunning(false);
        } else {
            setRunning(true);
        }
    });
    useInteractionEvent('A32NX_CHRONO_RST', () => {
        if (running) {
            setElapsedTime(0);
        } else {
            setElapsedTime(null);
        }
    });

    return (
        <text x="47" y="60" className="fontBig">{ltsTest ? '88:88' : getDisplayString(elapsedTime, running)}</text>
    );
};
