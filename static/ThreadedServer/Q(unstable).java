package Full;

import java.util.concurrent.*;

public class Q extends Front{

    public final String methodName;

    public final Object[] args;

    public final CountDownLatch resultReady;

    public Object result;

    public Q(String methodName, Object[] args) {
        this.methodName = methodName;
        this.args = args;
        this.resultReady = new CountDownLatch(1);
    }

    public void setResult(Object result) {
        this.result = result;
        resultReady.countDown();
    }

    public Object getResult() throws InterruptedException {
        resultReady.await();
        return result;
    }
    //public SynchronousQueue<Q> methodCalls = new SynchronousQueue<Q>();
}
