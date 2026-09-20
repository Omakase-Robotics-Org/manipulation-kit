from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.error_envelope import ErrorEnvelope
from ...models.soft_kill_response_200 import SoftKillResponse200
from ...types import Response


def _get_kwargs() -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/soft_kill",
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | SoftKillResponse200 | None:
    if response.status_code == 200:
        response_200 = SoftKillResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ErrorEnvelope | SoftKillResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | SoftKillResponse200]:
    """Latch the soft kill on every device and stop motion

     The COMPLETE emergency stop of everything this daemon owns, in one call: both arms, both grippers,
    both hands, the neck, the eyes, the slider and the chassis (zero velocity, navigation paused, light
    strip off). Do not implement a device-by-device kill above this route. Every latch is set before any
    stop is issued and the stops run concurrently, so one wedged device cannot delay another; the
    response reports each action as `ok`, `failed`, `timeout` or `unavailable`, with the milliseconds it
    took. The chassis actions are bounded at 1 s each and are skipped without any network I/O when the
    base is known to be absent, so this call returns promptly even against a wedged or missing base. The
    latch is sticky: motion verbs are refused with 409 until `/v1/release_soft_kill`. This is not a
    substitute for the physical emergency stop.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | SoftKillResponse200]
    """

    kwargs = _get_kwargs()

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | SoftKillResponse200 | None:
    """Latch the soft kill on every device and stop motion

     The COMPLETE emergency stop of everything this daemon owns, in one call: both arms, both grippers,
    both hands, the neck, the eyes, the slider and the chassis (zero velocity, navigation paused, light
    strip off). Do not implement a device-by-device kill above this route. Every latch is set before any
    stop is issued and the stops run concurrently, so one wedged device cannot delay another; the
    response reports each action as `ok`, `failed`, `timeout` or `unavailable`, with the milliseconds it
    took. The chassis actions are bounded at 1 s each and are skipped without any network I/O when the
    base is known to be absent, so this call returns promptly even against a wedged or missing base. The
    latch is sticky: motion verbs are refused with 409 until `/v1/release_soft_kill`. This is not a
    substitute for the physical emergency stop.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | SoftKillResponse200
    """

    return sync_detailed(
        client=client,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | SoftKillResponse200]:
    """Latch the soft kill on every device and stop motion

     The COMPLETE emergency stop of everything this daemon owns, in one call: both arms, both grippers,
    both hands, the neck, the eyes, the slider and the chassis (zero velocity, navigation paused, light
    strip off). Do not implement a device-by-device kill above this route. Every latch is set before any
    stop is issued and the stops run concurrently, so one wedged device cannot delay another; the
    response reports each action as `ok`, `failed`, `timeout` or `unavailable`, with the milliseconds it
    took. The chassis actions are bounded at 1 s each and are skipped without any network I/O when the
    base is known to be absent, so this call returns promptly even against a wedged or missing base. The
    latch is sticky: motion verbs are refused with 409 until `/v1/release_soft_kill`. This is not a
    substitute for the physical emergency stop.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | SoftKillResponse200]
    """

    kwargs = _get_kwargs()

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | SoftKillResponse200 | None:
    """Latch the soft kill on every device and stop motion

     The COMPLETE emergency stop of everything this daemon owns, in one call: both arms, both grippers,
    both hands, the neck, the eyes, the slider and the chassis (zero velocity, navigation paused, light
    strip off). Do not implement a device-by-device kill above this route. Every latch is set before any
    stop is issued and the stops run concurrently, so one wedged device cannot delay another; the
    response reports each action as `ok`, `failed`, `timeout` or `unavailable`, with the milliseconds it
    took. The chassis actions are bounded at 1 s each and are skipped without any network I/O when the
    base is known to be absent, so this call returns promptly even against a wedged or missing base. The
    latch is sticky: motion verbs are refused with 409 until `/v1/release_soft_kill`. This is not a
    substitute for the physical emergency stop.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | SoftKillResponse200
    """

    return (
        await asyncio_detailed(
            client=client,
        )
    ).parsed
